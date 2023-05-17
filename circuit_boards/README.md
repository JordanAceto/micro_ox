# Micro-Ox Circuit Boards

## The system is made up of a few independent PCBs

- VCO board
- VCF/VCA board
- Modulation board
- Power supply and input-output jack board

### These boards are connected to each other via IDC headers
- IDC header pinouts are chosen so it is impossible to plug them in wrong

---

## Each major system component has its own subdirectory with:
- All files needed to edit the schematic and PCB layout with KiCad
- Documentation files needed to build and troubleshoot the circuits

---

## The approximate current draw of the circuitboards is:

| Board     | +12V  | -12V  |
| --------- | ----- | ----- |
| main VCO  | 70mA  | 60mA  |
| VCF/VCA   | 40mA  | 40mA  |
| mod board | 50mA  | 20mA  |
|           |       |       |
| **TOTAL** | 160mA | 120mA |

---

## Some boards may need kludges (small trace cuts/fixes)
- look for a `pcb_kludges` subdirectory in a given circuitboard directory
- if this subdirectory exists, it will include instructions for the kludge in the form of a README file
- if you don't find this directory, assume that the PCB does not require any kludges
