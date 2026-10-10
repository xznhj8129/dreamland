# Part Editor View

This file covers the part-library editor mounted into the main window.

## Layout

```text
PartEditorView [fill]

    * LibraryEditor [split:h]
        SidebarList
        PropertyEditor
```

## Header

```text
Header [arrange:row]

    - Save
        on:click -> write the part's category file, data/parts/<category>.json

    - New
        on:click -> create a new part in the current category
        New part also available in the sidebar before selecting a part

    - Delete
        on:click -> ask for confirmation, then delete current part
```

## SidebarList

```text
SidebarList [fill]

    - searchable tree grouped by category; part names beneath each category
    - selecting a row rebuilds the editor panel for that part
    - library refresh keeps the current selection if the same category/name still exists
```

## PropertyEditor

```text
PropertyEditor [fill]

    * PartHeader
        - category
        - name

    * Properties
        - flat scalar fields
        - nested dict groups become nested labeled sections
        - optional fields the entry leaves out, blank ("blank = left out"): battery connector, ir_mohm_per_cell,
          lead_awg, lead_length_mm; esc mosfet_rds_on_ohm
```

### Field behavior

```text
Formatting
    - length fields are shown in mm where applicable
    - mass fields are shown in g
    - descriptive labels with units; battery dimensions only under Dimensions
    - wiring origins and outline sizes have labelled axis inputs
    - other arrays retain JSON entry

Save behavior
    - values are parsed back into the existing field type
    - a blank optional field is removed from the entry
    - the entry keeps its place in its file; save may move a part to a different category
    - save triggers the main app to reload library data

New behavior
    - new battery parts use the battery defaults
    - new esc parts use the esc defaults; prop, material and connector parts likewise
```
