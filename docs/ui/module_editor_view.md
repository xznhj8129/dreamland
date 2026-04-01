# Module Editor View

This file covers the module-library editor mounted into the main window.

## Layout

```text
ModuleEditorView [fill]

    * LibraryEditor [split:h]
        SidebarList
        PropertyEditor
```

## Header

```text
Header [arrange:row]

    - Save
        on:click -> write current module set to data/modules.json

    - New
        on:click -> create a new body-template-backed module

    - Delete
        on:click -> ask for confirmation, then delete current module
```

## SidebarList

```text
SidebarList [fill]

    - listbox of module names
    - selecting a row rebuilds the editor panel for that module
    - refresh_from_disk keeps the current selection if the same name still exists
```

## PropertyEditor

```text
PropertyEditor [fill]

    * ModuleHeader
        - name

    * Properties
        - flat scalar fields
        - nested dict groups become nested labeled sections
```

### Field behavior

```text
Formatting
    - length fields are shown in mm
    - mass fields are shown in g
    - list fields are shown as JSON text

Save behavior
    - values are parsed back into the existing field type
    - renaming the module changes the dictionary key on save
    - save triggers the main app to reload library data
```
