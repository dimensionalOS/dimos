# Cockpit presentation

The application keeps its existing DimOS manifest layout. Each panel title bar supports dragging (or Alt + arrow keys), minimizing, and the existing maximize/restore action. Minimized panels reappear from the workspace toolbar; Reset panels restores the initial layout. Position changes are session-local. The dark/light preference persists in browser storage.

Football camera uses the same frame as Duck camera. Its image and YOLO boxes still come from the paired server frame. The detection status remains below the feed.

The app registers its humancli and Teleop presentations through the DimOS panel registry. `web/src/cockpit/` adapts the pinned Apache-2.0 DimOS UI components. A Vite plugin substitutes the local frame for shared panels and maps shared CSS colors to theme variables. The shared framework checkout is unchanged.

Humancli retains DimOS's ChatLog, streamed rows, pending/retry handling, and send path. The transcript stays visible in Teleop mode. Input, send, and retry are disabled until Agent mode returns. Its logo and terminal colors follow humancli.

Focusing Teleop requests `set_mode(teleop)` through the existing control command channel, then waits for the mode stream acknowledgment before arming the existing TeleopMachine. Focus loss, minimization, a hidden tab, disconnect, and unmount release the lease and stop motion. Keyboard messages remain scoped to Teleop. No global drive-key listeners were added.

Validation on 2026-09-10: production build and TypeScript check passed; all 14 existing frontend tests passed. Live browser checks covered server-acknowledged mode switching, disabled/enabled chat input, camera dragging, panel reset, camera minimize/restore, Teleop disarm on minimize, and both themes. No browser errors were reported. Physics and services were not restarted for this frontend update.


## Follow-up layout and sign-out

Teleop sits above two square camera feeds, with the map below. The default world camera starts at the pitch view. The Three.js POV uses the panel aspect ratio; the native football image and its SVG detections use the same centered crop. Detection still runs on the complete native frame. Maximized camera views remain square and fit the viewport.

Clicking humancli requests Agent mode through the same control channel used by the mode buttons. The composer enables only after the mode stream acknowledges Agent mode.

Sign-out clears both site cookies (session and OAuth state), deletes the corresponding server-side session and pending OAuth state, revokes access to the match, and clears local join/profile state. A site cannot clear cookies belonging to github.com, so this does not sign the user out of GitHub itself.

Follow-up validation: 14 frontend tests and 12 edge tests passed, including session invalidation and deletion of both cookies. Type checks and production build passed. Live checks verified equal 180.125 px camera image dimensions, coincident image/overlay bounds, Agent mode acknowledgment, and a maximized 978 px square camera fitting inside the 1600 by 1100 viewport. The test duck was released.


## Rendering and account-choice correction

Humancli now uses the full native sender-prefix width in wide panels and stacked metadata in narrow panels, avoiding text overlap. Its ASCII banner scales as vector text. Bold and inline code are rendered as safe React text elements; transport-only reasoning/function-call markers are omitted. Live checks verified tool calls, tool results, formatted coordinates, and no row overlap at normal and maximized sizes.

GitHub authorization explicitly requests `prompt=select_account`, so an existing GitHub session presents account choice. This does not force re-entry of a GitHub password or clear GitHub-owned cookies.

All six ducks now spawn facing the pitch at x=-0.3 (red) or x=0.3 (blue), with y=1.8, 1.2, and 0.6. These positions are in the central passage outside the midfield sideline. The 0.5 m inter-duck clearance is unchanged. A real-policy test activated all six, verified upright standing for three seconds, and checked manual respawn; all twelve pitch-entry walking cases passed. Validation: 26 scene/physics tests, 16 frontend tests, and 12 edge tests, plus type checks and build.

Camera panel headers keep the title, renderer or duck selector, and window controls on one row. FPS appears over each feed. Drag the bottom-right handle to resize a panel, or focus the handle and use arrow keys. Reset panels restores the default layout. Native sensor images are 640×360; square defaults center-crop the image. YOLO processes the complete native frame. Duck Three.js FPS counts client draws; football FPS counts decoded detection images.

The current layout places a 220px-wide YOLO window over the upper-left world view. Duck camera occupies the full controls-column width. Both feeds preserve native simulation 16:9 framing with letterboxing when resized, rather than cropping. Expanded camera framing uses the same camera mount and vertical field of view. Physical camera resolution and field of view remain provisional according to Pollen's press kit, so 640×360 is a simulation configuration, not a verified hardware specification.

YOLO currently receives server-rendered MuJoCo RGB images, predicts sports-ball boxes on Omarchy, and sends JPEG images plus bounding-box JSON through the existing DimOS camera channels. The browser displays JPEG pixels and SVG boxes. Humancli currently receives raw images via observe; it does not subscribe to ball_detections.

Proposed agent integration: route each FootballObservation to its owning robot generation, retain the synchronized RGB-D observation, and expose a read-only get_detections skill through that robot's MCP server. Return timestamp, class, confidence, box and depth-derived position, reject stale or previous-generation observations, and avoid giving one duck another duck's detections. Add per-duck tracking only when persistent object IDs are needed. This integration is planned, not implemented.

Floating, expanded and minimized panels preserve their docked slot height so adjacent panels do not expand unexpectedly. Humancli requests Agent mode on pointer-down or keyboard focus and permits immediate local drafting; sending remains gated on the server mode update. Leaving the panel clears the local editing state.

Public connection incident: repeated account-level replacements caused automatic reconnecting tabs to take ownership back from each other. Edge diagnostics confirmed the replacement reason with small pending queues. PublicTransport now notifies its owning session of an intentional replacement, and LobbyApp closes that session to stop retries. Workspace shows a replacement notice. Ordinary disconnects retain SDK reconnection. Reload existing tabs to load this behavior. Diagnostics log only reasons and queue counters.

The marked pitch is now 2.6×1.6 m, centered at (0, 4.3). The surrounding floor, walls, rooms, goal dimensions and duck spawns remain unchanged. Goal centers move to x=±1.3 m; the ball spawn positions and pitch paint scale around midfield.

A Three.js wall board beside the team scoreboard displays the top eight all-time GitHub scorers. DimOS attributes goals to the last authenticated duck contact; own goals, anonymous contacts, ambiguous simultaneous contacts, and stale participant generations receive no personal credit. Physical goals still count for the team. GitHub numeric IDs are the database keys, and authenticated handles are displayed. SQLite state is stored in state/football-scorers.sqlite3 and survives scoreboard resets and service restarts. Database writes occur on the lobby worker thread. The cosmetic board is absent from native MuJoCo RGB and YOLO images.


Midfield recovery update: duck spawn and manual respawn positions are x=-0.6, 0, 0.6 at y=3.25 for red and y=5.35 for blue, facing the pitch. The scorer board is 2.2 by 1.15 metres with larger handle and goal text. Teleop includes individual drop buttons for all three pitch balls and the benchmark ball. Each requests a server-authoritative reset to (0, 4.3, 2.0), clears velocity and previous goal attribution, and falls under normal MuJoCo gravity. A busy drop position queues briefly until clear; stale or revoked-player requests expire. The app extends its own DimOS bridge command validator with drop_ball; shared framework code is unchanged.
