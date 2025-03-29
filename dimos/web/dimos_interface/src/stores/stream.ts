/**
 * Copyright 2025 Dimensional Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

import { writable, derived, get } from 'svelte/store';
import { simulationManager, simulationStore } from '../utils/simulation';
import { history } from './history';

interface StreamState {
  isVisible: boolean;
  url: string | null;
  isLoading: boolean;
  error: string | null;
  streamKey: string | null;
  availableStreams: string[];
}

interface TextStreamState {
  isStreaming: boolean;
  messages: string[];
  currentStream: EventSource | null;
  streamKey: string | null;
}

const initialState: StreamState = {
  isVisible: false,
  url: null,
  isLoading: false,
  error: null,
  streamKey: null,
  availableStreams: []
};

const initialTextState: TextStreamState = {
  isStreaming: false,
  messages: [],
  currentStream: null,
  streamKey: null
};

export const streamStore = writable<StreamState>(initialState);
export const textStreamStore = writable<TextStreamState>(initialTextState);
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
    const response = await fetch('http://0.0.0.0:5555/streams', {
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
      url: 'http://0.0.0.0:5555',
      streamKey,
      isLoading: false,
      error: null,
      availableStreams: (await fetchAvailableStreams()),
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

// Simple store to track active event sources
const textEventSources: Record<string, EventSource> = {};

export const connectTextStream = (key: string): void => {
  // Close existing stream if any
  if (textEventSources[key]) {
    textEventSources[key].close();
    delete textEventSources[key];
  }

  // Create new EventSource
  const eventSource = new EventSource(`http://0.0.0.0:5555/text_stream/${key}`);
  textEventSources[key] = eventSource;
  // Handle incoming messages
  eventSource.addEventListener('message', (event) => {
    // Append message to the last history entry
    history.update(h => {
      const lastEntry = h[h.length - 1];
      const newEntry = {
        ...lastEntry,
        outputs: [...lastEntry.outputs, event.data]
      };
      return [
        ...h.slice(0, -1),
        newEntry
      ];
    });
  });

  // Handle errors
  eventSource.onerror = (error) => {
    console.error('Stream error details:', {
      key,
      error,
      readyState: eventSource.readyState,
      url: eventSource.url
    });
    eventSource.close();
    delete textEventSources[key];
  };
};

export const disconnectTextStream = (key: string): void => {
  if (textEventSources[key]) {
    textEventSources[key].close();
    delete textEventSources[key];
  }
};

