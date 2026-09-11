# Character-select lobby

The entry screen behaves like choosing a playable character. The actual Microduck
model is the hero, with three distinct shell colors and a fourth spectator option.
The spectator preview uses the scene's room coordinates and live body poses.

Palette: deep blue #142443, panel blue #213758, cloud white #eff4ff,
Duck 1 gold #ffcf62, Duck 2 teal #62d8cb, Duck 3 violet #b49aff.
Typography: locally hosted Chakra Petch for titles and actions; system sans for
short instructions. Labels use sentence case. Duck numbers are identities, not
ornamental numbering. Availability and map knowledge have explicit text labels.

Desktop: title and occupancy above one row of three robot choices and a quieter
spectator choice. Narrow layouts become two columns, then one column. Each choice
is one keyboard-accessible action. The actual cockpit retains its working layout.

    Microduck World                             1 / 3 ducks occupied
    Choose your Microduck
    [ Gold robot ] [ Teal robot ] [ Violet robot ] [ Labeled room map ]
    [ Duck 1     ] [ Duck 2     ] [ Duck 3       ] [ Spectator        ]
    [ Map loaded ] [ Start fresh] [ Start fresh  ] [ Watch the world  ]

Review against the brief: avoid a marketing hero and generic gradient cards. Spend
visual emphasis on the recognizable robot portraits, with color indicating a real
in-world identity. Render portraits once with a temporary WebGL context and reuse
them as images; do not create three additional continuously rendering world feeds.
