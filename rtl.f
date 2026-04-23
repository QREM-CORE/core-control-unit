# 1. Submodules
# NOTE: lib/common-rtl submodule is not populated locally; resolve via the
# HSU repo which is the canonical populated copy in this workspace.
-f ../../HSU/hash-sampler-unit/lib/common-rtl/rtl.f

# 2. Local Packages
rtl/core_ctrl_pkg.sv

# 3. Local RTL
rtl/host_if.sv
rtl/kg_fsm.sv

# 4. Top Level
rtl/core_control_unit.sv
