import { writable, derived } from 'svelte/store';
import { simulationManager, simulationStore } from '../utils/simulation';

interface StreamState {
  isVisible: boolean;
  url: string | null;
  isLoading: boolean;
  error: string | null;
  streamKey: string | null;
  availableStreams: string[];
}

const initialState: StreamState = {
  isVisible: false,
  url: null,
  isLoading: false,
  error: null,
  streamKey: null,
  availableStreams: []
};

export const streamStore = writable<StreamState>(initialState);
// Derive stream state from both stores
export const combinedStreamState = derived(
  [streamStore, simulationStore],
  ([$stream, $simulation]) => ({
    ...$stream,
    isLoading: $stream.isLoading || $simulation.isConnecting,
    error: $stream.error || $simulation.error
  })
);

// Function to fetch available streams
async function fetchAvailableStreams(): Promise<string[]> {
  try {
    const response = await fetch('http://localhost:5555/streams', {
      headers: {
        'Accept': 'application/json'
      }
    });
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    const data = await response.json();
    return data.streams;
  } catch (error) {
    console.error('Failed to fetch available streams:', error);
    return [];
  }
}

// Initialize store with available streams
fetchAvailableStreams().then(streams => {
  streamStore.update(state => ({ ...state, availableStreams: streams }));
});

export const showStream = async (streamKey?: string) => {
  streamStore.update(state => ({ ...state, isLoading: true, error: null }));
  
  try {
    // If no streamKey provided, get available streams and use the first one
    if (!streamKey) {
      const streams = await fetchAvailableStreams();
      if (streams.length === 0) {
        throw new Error('No video streams available');
      }
      streamKey = streams[0];
    }

    streamStore.set({
      isVisible: true,
      url: 'http://localhost:5555',
      streamKey,
      isLoading: false,
      error: null,
      availableStreams: (await fetchAvailableStreams())
    });

  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Failed to connect to stream';
    streamStore.update(state => ({
      ...state,
      isLoading: false,
      error: errorMessage
    }));
    throw error;
  }
};

export const hideStream = async () => {
  await simulationManager.stopSimulation();
  streamStore.set(initialState);
};
