<script lang="ts">
  import Ps1 from './components/Ps1.svelte';
  import Input from './components/Input.svelte';
  import History from './components/History.svelte';
  import StreamViewer from './components/StreamViewer.svelte';
  import VoiceButton from './components/VoiceButton.svelte';
  import { theme } from './stores/theme';
  import { history } from './stores/history';

  const handleVoiceCommand = async (event: CustomEvent) => {
    if (event.detail.success) {
      // Show voice processing message
      history.update(h => [...h, {
        command: '[voice command]',
        outputs: ['Processing voice command...']
      }]);

      // The actual command will be processed by the agent through the audio pipeline
      // and will appear in the text stream
    } else {
      history.update(h => [...h, {
        command: '[voice command]',
        outputs: [`Error: ${event.detail.error}`]
      }]);
    }
  };
</script>

<svelte:head>
  {#if import.meta.env.VITE_TRACKING_ENABLED === 'true'}
    <script
      async
      defer
      data-website-id={import.meta.env.VITE_TRACKING_SITE_ID}
      src={import.meta.env.VITE_TRACKING_URL}
    ></script>
  {/if}
</svelte:head>

<main
  class="h-full border-2 rounded-md p-4 overflow-auto text-xs sm:text-sm md:text-base"
  style={`background-color: ${$theme.background}; color: ${$theme.foreground}; border-color: ${$theme.green};`}
>
  <StreamViewer />
  <History />

  <div class="flex flex-col md:flex-row">
    <Ps1 />
    <Input />
  </div>
</main>

<VoiceButton on:voiceCommand={handleVoiceCommand} />
