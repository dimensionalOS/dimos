<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { streamStore } from '../stores/stream';

  let errorMessages: Record<string, string | null> = {};
  let retryCount: Record<string, number> = {};
  let retryTimers: Record<string, number | null> = {};
  let timestamps: Record<string, number> = {};
  const TOTAL_TIMEOUT = 120000; // 2 minutes
  const RETRY_INTERVAL = 2000; // Retry every 2 seconds
  const MAX_RETRIES = Math.floor(TOTAL_TIMEOUT / RETRY_INTERVAL);

  // Define initial state to reset the stream
  const initialState = {
    isVisible: false,
    url: null,
    streamKeys: [],
    availableStreams: []
  };

  function clearRetryTimer(streamKey: string) {
    if (retryTimers[streamKey] !== null) {
      clearTimeout(retryTimers[streamKey]);
      retryTimers[streamKey] = null;
    }
  }

  function retryConnection(streamKey: string) {
    if (!retryCount[streamKey]) retryCount[streamKey] = 0;
    
    if (retryCount[streamKey] < MAX_RETRIES) {
      retryCount[streamKey]++;
      const timeLeft = TOTAL_TIMEOUT - (retryCount[streamKey] * RETRY_INTERVAL);
      errorMessages[streamKey] = `Connection attempt ${retryCount[streamKey]}/${MAX_RETRIES}... (${Math.ceil(timeLeft / 1000)}s remaining)`;
      
      // Update timestamp to force a new connection attempt
      timestamps[streamKey] = Date.now();
      
      clearRetryTimer(streamKey);
      retryTimers[streamKey] = setTimeout(() => retryConnection(streamKey), RETRY_INTERVAL);
    } else {
      errorMessages[streamKey] = 'Failed to connect to stream. Please check if the Robot() is running and sending data to RobotWebInterface.';
    }
  }

  function handleError(streamKey: string) {
    if (!retryCount[streamKey] || retryCount[streamKey] === 0) {
      retryConnection(streamKey);
    }
  }

  function handleLoad(streamKey: string) {
    errorMessages[streamKey] = null;
    retryCount[streamKey] = 0;
    clearRetryTimer(streamKey);
  }

  function stopStream() {
    Object.keys(retryTimers).forEach(key => clearRetryTimer(key));
    streamStore.set(initialState);
  }

  // Reset error state when stream URL changes
  $: if ($streamStore.url && $streamStore.streamKeys) {
    $streamStore.streamKeys.forEach(key => {
      errorMessages[key] = null;
      retryCount[key] = 0;
      clearRetryTimer(key);
      timestamps[key] = Date.now();
    });
  }

  onDestroy(() => {
    Object.keys(retryTimers).forEach(key => clearRetryTimer(key));
  });

  // Compute current URLs with timestamps to prevent caching
  $: streamUrls = $streamStore.streamKeys.map(key => ({
    key,
    url: $streamStore.url ? `${$streamStore.url}/video_feed/${key}?t=${timestamps[key] || Date.now()}` : null
  }));

  // Calculate grid layout based on number of streams
  $: gridCols = Math.ceil(Math.sqrt($streamStore.streamKeys.length));
  $: gridRows = Math.ceil($streamStore.streamKeys.length / gridCols);
</script>

<div class="stream-viewer" class:visible={$streamStore.isVisible}>
  <div class="stream-container" style="--grid-cols: {gridCols}; --grid-rows: {gridRows};">
    <div class="stream-title">Unitree Robot Feeds</div>
    {#if $streamStore.isVisible}
      {#each streamUrls as {key, url}}
        <div class="stream-cell">
          {#if url}
            <img
              src={url}
              alt={`Robot video stream ${key}`}
              on:error={() => handleError(key)}
              on:load={() => handleLoad(key)}
            />
          {/if}
          {#if errorMessages[key]}
            <div class="error-message">
              {errorMessages[key]}
            </div>
          {/if}
        </div>
      {/each}
    {/if}
    <button class="close-btn" on:click={stopStream}>
      Stop Streams
    </button>
  </div>
</div>

<style>
  .stream-viewer {
    position: fixed;
    top: 20px;
    right: 20px;
    z-index: 1000;
    display: none;
    background: rgba(0, 0, 0, 0.8);
    padding: 10px;
    border-radius: 8px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  }

  .visible {
    display: block;
  }

  .stream-container {
    position: relative;
    width: calc(640px * var(--grid-cols));
    max-width: 90vw;
    display: grid;
    grid-template-columns: repeat(var(--grid-cols), 1fr);
    gap: 10px;
    padding: 10px;
  }

  .stream-cell {
    position: relative;
    width: 100%;
    aspect-ratio: 4/3;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  img {
    width: 100%;
    height: 100%;
    object-fit: contain;
  }

  .error-message {
    position: absolute;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    background: rgba(0, 0, 0, 0.8);
    color: #ff4444;
    padding: 10px;
    border-radius: 4px;
    text-align: center;
    font-size: 14px;
    max-width: 90%;
  }

  .stream-title {
    position: absolute;
    top: -30px;
    left: 0;
    color: white;
    font-size: 16px;
    font-weight: bold;
  }

  .close-btn {
    position: absolute;
    top: -8px;
    right: -8px;
    padding: 4px 8px;
    border-radius: 4px;
    background: rgba(255, 255, 255, 0.9);
    border: none;
    color: #000;
    font-size: 14px;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
  }
</style> 