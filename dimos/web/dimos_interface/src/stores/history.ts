import { writable } from 'svelte/store';
import type { Command } from '../interfaces/command';

export const history = writable<Array<Command>>(
  JSON.parse(localStorage.getItem('history') || '[]'),
);

history.subscribe((value) => {
  localStorage.setItem('history', JSON.stringify(value));
});
