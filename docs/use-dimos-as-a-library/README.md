# Use dimOS as a library

Your own Python project can depend on an installed dimOS instead of living
inside this repository. You import the packages dimOS ships, reuse its message
types and its module and blueprint APIs, and register your own extensions from
your own distribution. Nothing in these tutorials asks you to edit this
repository, patch its registries or regenerate its built-in schemas.

Three things you can add from the outside, each as a small package installed
next to dimOS:

- [Add a message in your own project](/docs/use-dimos-as-a-library/external-message.md):
  keep `.msg` definitions in your package and reuse the installed `Header`.
- [Add a module in your own project](/docs/use-dimos-as-a-library/external-module.md):
  a `Module` subclass with configuration, lifecycle and typed streams.
- [Run your modules with a blueprint](/docs/use-dimos-as-a-library/external-blueprint.md):
  compose those modules, register a runnable name and start it with `dimos run`.

The module and blueprint tutorials build the same package, `garden-monitor`, so
follow the module tutorial first if you want to run them in order.

## Before you start

- Python 3.12 and `pip`.
- An environment with dimOS installed. Follow
  [installation](/docs/installation/index.md) for a released dimOS or for a
  source checkout.

Check the installed versions:

```sh
python -c "from importlib.metadata import version; print(version('dimos'), version('dimos-cdrgen'))"
```

None of the tutorials need a robot, hardware, a network service or an API key.
The external package builds the same way with a released dimOS and with a local
checkout; only the way dimOS itself was installed differs.

## For dimOS developers

Changing the repository itself, including its built-in messages, is covered by
the developer documentation:
[CDR generation in dimOS](/docs/development/message-source-packages.md).
