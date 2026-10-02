# GUIBuilder UI Layout

This directory is split by window scope instead of keeping the whole UI in one file.

## Shared notation

```text
Component [attr, attr]          Layout/behavior properties
    ui:dropdown/radio/textbox/etc
    arrange:column/row
    on:event -> action
    bind:state_key
    enum:name -> { A, B, C }

    * Child                     Container / structural child
    - Item                      Direct interactive control
```

### Attribute glossary

| Attribute     | Meaning                                  |
|---------------|------------------------------------------|
| `docked:edge` | Pinned to one edge of parent             |
| `fill`        | Expands to fill remaining space          |
| `split:h`     | Horizontal pane split                    |
| `split:v`     | Vertical pane split                      |
| `scrollable`  | Has scrollable content area              |
| `switchable`  | Multiple modes, one visible at a time    |
| `copyable`    | Text can be selected and copied          |
| `editable`    | User can directly edit values            |
| `readonly`    | Display only; user cannot type arbitrary |
| `bind:key`    | Reads/writes shared application state    |

## Files

- [main_window.md](docs/ui/main_window.md)
- [configuration_view.md](docs/ui/configuration_view.md)
- [aero_analysis_view.md](docs/ui/aero_analysis_view.md)
- [vlm_view.md](docs/ui/vlm_view.md)
- [cad_view.md](docs/ui/cad_view.md)
- [part_editor_view.md](docs/ui/part_editor_view.md)
