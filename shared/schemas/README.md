# shared/schemas

The machine-readable mirror of the message contract described in
[../../docs/message-schemas.md](../../docs/message-schemas.md). That document is the human
source of truth. The files here are what code imports.

- `*.schema.json` are JSON Schema (draft 2020-12) definitions, language neutral. The
  visualization and any validation tooling read these.
- `schemas.py` provides the same messages as typed Python dataclasses for the navigation
  brain, with `to_dict` / `from_dict` helpers and the ASCII serial line helpers for
  `DriveCommand` and `CarTelemetry`.

Keep all three in sync. If you change a field, change `docs/message-schemas.md` first,
then update the JSON Schema and the dataclass together.

Field names are identical across the firmware (C++), the brain (Python), and the viewer
(JS). That is the whole point: one contract, reused everywhere.
