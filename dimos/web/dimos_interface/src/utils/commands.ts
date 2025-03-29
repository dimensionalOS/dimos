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

import packageJson from '../../package.json';
import themes from '../../themes.json';
import { get } from 'svelte/store';
import { history } from '../stores/history';
import { theme } from '../stores/theme';
import { showStream, hideStream } from '../stores/stream';
import { simulationStore, type SimulationState } from '../utils/simulation';

let bloop: string | null = null;
const hostname = window.location.hostname;
const bleepbloop = import.meta.env.VITE_ENV_VARIABLE;
const xXx_VaRiAbLeOfDeAtH_xXx = "01011010 01000100 01000110 01110100 01001101 00110010 00110100 01101011 01100001 01010111 00111001 01110101 01011000 01101010 01000101 01100111 01011001 01111010 01000010 01110100 01010000 00110011 01010110 01010101 01001101 01010111 00110101 01101110";
function someRandomFunctionIforget(binary: string): string {
  return atob(binary.split(' ').map(bin => String.fromCharCode(parseInt(bin, 2))).join(''));
}
const var23temp_pls_dont_touch = someRandomFunctionIforget(xXx_VaRiAbLeOfDeAtH_xXx);
const magic_url = "https://agsu5pgehztgo2fuuyip6dwuna0uneua.lambda-url.us-east-2.on.aws/";

type CommandResult = string | {
  type: 'STREAM_START';
  streamKey: string;
  initialMessage: string;
};

// Function to fetch available text stream keys
async function fetchTextStreamKeys(): Promise<string[]> {
  try {
    const response = await fetch('/text_streams');
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    const data = await response.json();
    return data.streams;
  } catch (error) {
    console.error('Failed to fetch text stream keys:', error);
    return [];
  }
}

// Cache the text stream keys
let textStreamKeys: string[] = [];
fetchTextStreamKeys().then(keys => {
  textStreamKeys = keys;
});

export const commands: Record<string, (args: string[]) => Promise<CommandResult> | CommandResult> = {
  help: () => 'Available commands: ' + Object.keys(commands).join(', '),
  hostname: () => hostname,
  whoami: () => 'guest',
  join: () => 'Actively recruiting all-star contributors. Build the future of dimensional computing with us. Reach out to build@dimensionalOS.com',
  date: () => new Date().toLocaleString(),
  vi: () => `why use vi? try 'vim'`,
  emacs: () => `why use emacs? try 'vim'`,
  echo: (args: string[]) => args.join(' '),
  sudo: (args: string[]) => {
    window.open('https://www.youtube.com/watch?v=dQw4w9WgXcQ');

    return `Permission denied: unable to run the command '${args[0]}'. Not based.`;
  },
  theme: (args: string[]) => {
    const usage = `Usage: theme [args].
    [args]:
      ls: list all available themes
      set: set theme to [theme]

    [Examples]:
      theme ls
      theme set gruvboxdark
    `;
    if (args.length === 0) {
      return usage;
    }

    switch (args[0]) {
      case 'ls': {
        const themeNames = themes.map((t) => t.name.toLowerCase());
        const formattedThemes = themeNames
          .reduce((acc: string[], theme: string, i: number) => {
            const readableTheme = theme.replace(/([a-z])([A-Z])/g, '$1 $2').toLowerCase();
            const paddedTheme = readableTheme.padEnd(30, ' '); // Increased padding to 30 chars
            if (i % 5 === 4 || i === themeNames.length - 1) {
              return [...acc, paddedTheme + '\n'];
            }
            return [...acc, paddedTheme];
          }, [])
          .join('');

        return formattedThemes;
      }

      case 'set': {
        if (args.length !== 2) {
          return usage;
        }

        const selectedTheme = args[1];
        const t = themes.find((t) => t.name.toLowerCase() === selectedTheme);

        if (!t) {
          return `Theme '${selectedTheme}' not found. Try 'theme ls' to see all available themes.`;
        }

        theme.set(t);

        return `Theme set to ${selectedTheme}`;
      }

      default: {
        return usage;
      }
    }
  },
  clear: () => {
    history.set([]);

    return '';
  },
  contact: () => {
    window.open(`mailto:${packageJson.author.email}`);

    return `Opening mailto:${packageJson.author.email}...`;
  },
  donate: () => {
    window.open(packageJson.donate.url, '_blank');

    return 'Opening donation url...';
  },
    invest: () => {
    window.open(packageJson.funding.url, '_blank');

    return 'Opening SAFE url...';
  },
  weather: async (args: string[]) => {
    const city = args.join('+');

    if (!city) {
      return 'Usage: weather [city]. Example: weather Brussels';
    }

    const weather = await fetch(`https://wttr.in/${city}?ATm`);

    return weather.text();
  },

  ls: () => {
    return 'whitepaper.txt';
  },
    cd: () => {
    return 'Permission denied: you are not that guy, pal';
  },
  curl: async (args: string[]) => {
    if (args.length === 0) {
      return 'curl: no URL provided';
    }

    const url = args[0];

    try {
      const response = await fetch(url);
      const data = await response.text();

      return data;
    } catch (error) {
      return `curl: could not fetch URL ${url}. Details: ${error}`;
    }
  },
  banner: () => `

██████╗ ██╗███╗   ███╗███████╗███╗   ██╗███████╗██╗ ██████╗ ███╗   ██╗ █████╗ ██╗     
██╔══██╗██║████╗ ████║██╔════╝████╗  ██║██╔════╝██║██╔═══██╗████╗  ██║██╔══██╗██║     
██║  ██║██║██╔████╔██║█████╗  ██╔██╗ ██║███████╗██║██║   ██║██╔██╗ ██║███████║██║     
██║  ██║██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║╚════██║██║██║   ██║██║╚██╗██║██╔══██║██║     
██████╔╝██║██║ ╚═╝ ██║███████╗██║ ╚████║███████║██║╚██████╔╝██║ ╚████║██║  ██║███████╗
╚═════╝ ╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝v${packageJson.version}

Powering generalist robotics 

Type 'help' to see list of available commands.
`,
  vim: async (args: string[])=> {
    const filename = args.join(' ');

    if (!filename) {
      return 'Usage: vim [filename]. Example: vim robbie.txt';
    }

    if (filename === "whitepaper.txt") {
      if (bloop === null) {
        return `File ${filename} is encrypted. Use 'vim -x ${filename}' to access.`;
      } else {
        return `Incorrect encryption key for ${filename}. Access denied.`;
      }
    }

      if (args[0] === '-x' && args[1] === "whitepaper.txt") {
        const bloop_master = prompt("Enter encryption key:");

        if (bloop_master === var23temp_pls_dont_touch) {
          try {
            const response = await fetch(magic_url, {
              method: 'POST',
              headers: {
                'Content-Type': 'application/json',
              },
              body: JSON.stringify({key: bloop_master}),
            });

            if (response.status === 403) {
              return "Access denied. You are not worthy.";
            }

            if (response.ok) {
              const manifestoText = await response.text();
              bloop = bloop_master;
              return manifestoText;
            } else {
              return "Failed to retrieve. You are not worthy.";
            }
          } catch (error) {
            return `Error: ${error.message}`;
          }
        } else {
          return "Access denied. You are not worthy.";
        }
      }

    return `bash: ${filename}: No such file`;
  },
  simulate: (args: string[]) => {
    if (args.length === 0) {
      return 'Usage: simulate [start|stop] - Start or stop the simulation stream';
    }

    const command = args[0].toLowerCase();

    if (command === 'stop') {
      hideStream();
      return 'Stream stopped.';
    }

    if (command === 'start') {
      showStream();
      return 'Starting simulation stream... Use "simulate stop" to end the stream';
    }

    return 'Invalid command. Use "simulate start" to begin or "simulate stop" to end.';
  },
  control: async (args: string[]) => {
    if (args.length === 0) {
      return 'Usage: control [joint_positions] - Send comma-separated joint positions to control the robot\nExample: control 0,0,0.5,1,0.3';
    }

    const state = get(simulationStore) as SimulationState;
    if (!state.connection) {
      return 'Error: No active simulation. Use "simulate start" first.';
    }

    const jointPositions = args.join(' ');
    
    try {
      const jointPositionsArray = jointPositions.split(',').map(x => parseFloat(x.trim()));
      const response = await fetch(`${state.connection.url}/control?t=${Date.now()}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'application/json'
        },
        body: JSON.stringify({ joint_positions: jointPositionsArray })
      });

      const data = await response.json();
      
      if (response.ok) {
        return `${data.message} ✓`;
      } else {
        return `Error: ${data.message}`;
      }
    } catch (error: unknown) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      return `Failed to send command: ${errorMessage}. Make sure the simulator is running.`;
    }
  },
  unitree: async (args: string[]) => {
    if (args.length === 0) {
      return 'Usage: unitree [status|start_stream|stop_stream|command <text>] - Interact with the Unitree API';
    }

    const subcommand = args[0].toLowerCase();
    
    if (subcommand === 'status') {
      try {
        const response = await fetch('/unitree/status');
        if (!response.ok) {
          throw new Error(`Server returned ${response.status}`);
        }
        const data = await response.json();
        return `Unitree API Status: ${data.status}`;
      } catch (error: unknown) {
        const message = error instanceof Error ? error.message : 'Server unreachable';
        return `Failed to get Unitree status: ${message}. Make sure the API server is running.`;
      }
    }

    if (subcommand === 'start_stream') {
      try {
        showStream();
        return 'Starting Unitree video stream... Use "unitree stop_stream" to end the stream';
      } catch (error: unknown) {
        const message = error instanceof Error ? error.message : 'Server unreachable';
        return `Failed to start video stream: ${message}. Make sure the API server is running.`;
      }
    }

    if (subcommand === 'stop_stream') {
      hideStream();
      return 'Stopped Unitree video stream.';
    }
    
    if (subcommand === 'command') {
      if (args.length < 2) {
        return 'Usage: unitree command <text> - Send a command to the Unitree API';
      }
      
      const commandText = args.slice(1).join(' ');
      
      try {
        // Ensure we have the text stream keys
        if (textStreamKeys.length === 0) {
          textStreamKeys = await fetchTextStreamKeys();
        }

        const response = await fetch('/unitree/command', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ command: commandText })
        });
        
        if (!response.ok) {
          throw new Error(`Server returned ${response.status}`);
        }

        return {
          type: 'STREAM_START' as const,
          streamKey: textStreamKeys[0], // Using the first available text stream
          initialMessage: `Command sent: ${commandText}\nPlanningAgent output...`
        };
        
      } catch (error) {
        const message = error instanceof Error ? error.message : 'Server unreachable';
        return `Failed to send command: ${message}. Make sure the API server is running.`;
      }
    }
    
    return 'Invalid subcommand. Available subcommands: status, start_stream, stop_stream, command';
  },
};
