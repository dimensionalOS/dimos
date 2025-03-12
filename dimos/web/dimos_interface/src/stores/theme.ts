import { writable } from 'svelte/store';
import themes from '../../themes.json';
import type { Theme } from '../interfaces/theme';

const defaultColorscheme: Theme = themes.find((t) => t.name === 'DimOS')!;

export const theme = writable<Theme>(
  JSON.parse(
    localStorage.getItem('colorscheme') || JSON.stringify(defaultColorscheme),
  ),
);

theme.subscribe((value) => {
  localStorage.setItem('colorscheme', JSON.stringify(value));
});
