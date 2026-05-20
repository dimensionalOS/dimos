import { SKY }        from './data/sky.js';
import { TAGS }       from './data/tags.js';
import { GROUPS }     from './data/groups.js';
import { LIGHTS }     from './data/lights.js';
import { PRIMITIVES } from './data/structure.js';
import { ASSETS }     from './data/objects.js';

export default async function build({ loadLevel }) {
  await loadLevel({
    version: '2.0',
    worldKey: 'default',
    tags: TAGS,
    primitives: PRIMITIVES,
    assets: ASSETS,
    lights: LIGHTS,
    groups: GROUPS,
    sceneSettings: { sky: SKY },
  });

  return {
    embodiment: null,
    spawnPoint: { x: 2, y: 0.5, z: 3 },
  };
}
