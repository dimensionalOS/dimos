import { writable, get } from 'svelte/store';

interface SimulationConnection {
    url: string;
    instanceId: string;
    expiresAt: number;
}

export interface SimulationState {
    connection: SimulationConnection | null;
    isConnecting: boolean;
    error: string | null;
    lastActivityTime: number;
}

const initialState: SimulationState = {
    connection: null,
    isConnecting: false,
    error: null,
    lastActivityTime: 0
};

export const simulationStore = writable<SimulationState>(initialState);

class SimulationError extends Error {
    constructor(message: string) {
        super(message);
        this.name = 'SimulationError';
    }
}

export class SimulationManager {
    private static readonly PROD_API_ENDPOINT = 'https://0rqz7w5rvf.execute-api.us-east-2.amazonaws.com/default/getGenesis';
    private static readonly DEV_API_ENDPOINT = '/api';  // This will be handled by Vite's proxy
    private static readonly MAX_RETRIES = 3;
    private static readonly RETRY_DELAY = 1000;
    private static readonly INACTIVITY_TIMEOUT = 5 * 60 * 1000; // 5 minutes in milliseconds
    private inactivityTimer: NodeJS.Timeout | null = null;

    private get apiEndpoint(): string {
        return import.meta.env.DEV ? SimulationManager.DEV_API_ENDPOINT : SimulationManager.PROD_API_ENDPOINT;
    }

    private async fetchWithRetry(url: string, options: RequestInit = {}, retries = SimulationManager.MAX_RETRIES): Promise<Response> {
        try {
            const response = await fetch(url, {
                ...options,
                headers: {
                    ...options.headers,
                    'Content-Type': 'application/json',
                    'Accept': 'application/json'
                }
            });

            if (import.meta.env.DEV && !response.ok) {
                console.error('Request failed:', {
                    status: response.status,
                    statusText: response.statusText,
                    headers: Object.fromEntries(response.headers.entries()),
                    url
                });
            }

            if (!response.ok) {
                throw new SimulationError(`HTTP error! status: ${response.status} - ${response.statusText}`);
            }
            return response;
        } catch (error) {
            if (retries > 0) {
                console.warn(`Request failed, retrying... (${retries} attempts left)`);
                await new Promise(resolve => setTimeout(resolve, SimulationManager.RETRY_DELAY));
                return this.fetchWithRetry(url, options, retries - 1);
            }
            throw error;
        }
    }

    private startInactivityTimer() {
        if (this.inactivityTimer) {
            clearTimeout(this.inactivityTimer);
        }

        this.inactivityTimer = setTimeout(async () => {
            const state = get(simulationStore);
            const now = Date.now();
            if (state.lastActivityTime && (now - state.lastActivityTime) >= SimulationManager.INACTIVITY_TIMEOUT) {
                await this.stopSimulation();
            }
        }, SimulationManager.INACTIVITY_TIMEOUT);
    }

    private updateActivityTime() {
        simulationStore.update(state => ({
            ...state,
            lastActivityTime: Date.now()
        }));
        this.startInactivityTimer();
    }

    async requestSimulation(): Promise<SimulationConnection> {
        simulationStore.update(state => ({ ...state, isConnecting: true, error: null }));
        
        try {
            // Request instance allocation
            const response = await this.fetchWithRetry(this.apiEndpoint, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    user_id: 'user-' + Date.now()
                })
            });

            const instanceInfo = await response.json();
            
            if (import.meta.env.DEV) {
                console.log('API Response:', instanceInfo);
            }

            if (!instanceInfo.instance_id || !instanceInfo.public_ip || !instanceInfo.port) {
                throw new SimulationError(
                    `Invalid API response: Missing required fields. Got: ${JSON.stringify(instanceInfo)}`
                );
            }

            // In development, use direct HTTP to EC2. In production, use HTTPS through ALB
            const connection = {
                instanceId: instanceInfo.instance_id,
                url: import.meta.env.DEV
                    ? `http://${instanceInfo.public_ip}:${instanceInfo.port}`
                    : `https://sim.dimensionalos.com`,
                expiresAt: Date.now() + SimulationManager.INACTIVITY_TIMEOUT
            };

            if (import.meta.env.DEV) {
                console.log('Creating stream connection:', {
                    instanceId: connection.instanceId,
                    url: connection.url,
                    isDev: true,
                    expiresAt: new Date(connection.expiresAt).toISOString()
                });
            }

            simulationStore.update(state => ({
                ...state,
                connection,
                isConnecting: false,
                lastActivityTime: Date.now()
            }));

            this.startInactivityTimer();
            return connection;

        } catch (error) {
            const errorMessage = error instanceof Error ? error.message : 'Failed to request simulation';
            simulationStore.update(state => ({
                ...state,
                isConnecting: false,
                error: errorMessage
            }));
            
            if (import.meta.env.DEV) {
                console.error('Simulation request failed:', error);
            }
            
            throw error;
        }
    }

    async stopSimulation() {
        const state = get(simulationStore);
        if (state.connection) {
            try {
                await this.fetchWithRetry(this.apiEndpoint, {
                    method: 'DELETE',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        instance_id: state.connection.instanceId
                    })
                });
            } catch (error) {
                console.error('Error releasing instance:', error);
            }
        }

        if (this.inactivityTimer) {
            clearTimeout(this.inactivityTimer);
            this.inactivityTimer = null;
        }

        simulationStore.set(initialState);
    }
}

export const simulationManager = new SimulationManager();