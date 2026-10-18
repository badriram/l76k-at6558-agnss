# Predicted GPS orbit data for AT6558 / L76K receivers

`gps-7d.bin`: CASIC MSG-GPSEPH frames, one per GPS satellite every 2 hours for 7 days,
made from the newest IGS ultra-rapid orbit by `tools/predict` on the main branch.
Pick each satellite's frame nearest now with `casic_eph_select()` and send those after
AID-INI. `gps-7d.json` says when and from what it was made, with a self-check against
the satellites' own broadcasts. Rebuilt four times a day; this branch holds only the latest.

Orbit and clock data: International GNSS Service (IGS), via BKG. MIT licence (see LICENSE).
