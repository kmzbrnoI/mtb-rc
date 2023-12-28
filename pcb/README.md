MTB-RC PCB
==========

## Design

Schematics & PCB are designed in KiCad 7.

## Production

PCB is prepared to be automatically assembled in [JLCPCB](https://jlcpcb.com/).
SMD parts on **bottom** side should be assembled. Each SMD part has its `LCSC_ITEM`
attribute set.

```bash
$ make fab
```

## License

Content of this directory is provided under [Creative Commons
Attribution-ShareAlike 4.0
License](https://creativecommons.org/licenses/by-sa/4.0/) as openhardware
project. You may download any data, contribute to the project, create PCB
yourself or even sell it yourself.
