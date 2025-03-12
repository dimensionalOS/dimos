<script lang="ts">
  import { onMount, onDestroy } from 'svelte';
  import { streamStore } from '../stores/stream';

  let errorMessage: string | null = null;
  let retryCount = 0;
  let retryTimer: number | null = null;
  let timestamp = Date.now();
  const TOTAL_TIMEOUT = 120000; // 2 minutes
  const RETRY_INTERVAL = 2000; // Retry every 2 seconds
  const MAX_RETRIES = Math.floor(TOTAL_TIMEOUT / RETRY_INTERVAL);

  // Define initial state to reset the stream
  const initialState = {
    isVisible: false,
    url: null,
    streamKey: null
  };

  function clearRetryTimer() {
    if (retryTimer !== null) {
      clearTimeout(retryTimer);
      retryTimer = null;
    }
  }

  function retryConnection() {
    if (retryCount < MAX_RETRIES) {
      retryCount++;
      const timeLeft = TOTAL_TIMEOUT - (retryCount * RETRY_INTERVAL);
      errorMessage = `Connection attempt ${retryCount}/${MAX_RETRIES}... (${Math.ceil(timeLeft / 1000)}s remaining)`;
      
      // Update timestamp to force a new connection attempt
      timestamp = Date.now();
      
      clearRetryTimer();
      retryTimer = setTimeout(retryConnection, RETRY_INTERVAL);
    } else {
      errorMessage = 'Failed to connect to stream. Please check if the Robot() is running and sending data to RobotWebInterface.';
    }
  }

  function handleError() {
    if (retryCount === 0) {
      retryConnection();
    }
  }

  function handleLoad() {
    errorMessage = null;
    retryCount = 0;
    clearRetryTimer();
  }

  function stopStream() {
    clearRetryTimer();
    streamStore.set(initialState);
  }

  // Reset error state when stream URL changes
  $: if ($streamStore.url) {
    errorMessage = null;
    retryCount = 0;
    clearRetryTimer();
    timestamp = Date.now();
  }

  onDestroy(() => {
    clearRetryTimer();
  });

  // Compute the current URL with timestamp to prevent caching
  $: currentUrl = $streamStore.url && $streamStore.streamKey 
    ? `${$streamStore.url}/video_feed/${$streamStore.streamKey}?t=${timestamp}` 
    : null;
</script>

<div class="stream-viewer" class:visible={$streamStore.isVisible}>
  <div class="stream-container">
    <div class="stream-title">Unitree Robot Feed</div>
    {#if $streamStore.isVisible && currentUrl}
      <img
        src={currentUrl}
        alt="Robot video stream"
        on:error={handleError}
        on:load={handleLoad}
      />
    {/if}
    {#if errorMessage}
      <div class="error-message">
        {errorMessage}
      </div>
    {/if}
    <button class="close-btn" on:click={stopStream}>
      Stop Stream
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
    width: 640px;
    height: 480px;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  img {
    max-width: 100%;
    max-height: 100%;
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