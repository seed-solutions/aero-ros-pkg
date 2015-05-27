(load "package://aero_2015_06_drc/euslisp/include-aero-drc-valve-task.l")

(make-drc-testbed-models-with-aero)
(reset-env-to *drc-valve* (deg2rad -135))
