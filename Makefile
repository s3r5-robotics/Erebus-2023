path:; node ./scripts/pathing.mjs
compile:; node ./scripts/compile.mjs
all:; make path; make compile

# `make path` will update the path in all `/src/` files
# `make compile` will compile all `/src/` files to `main.py`
# `make all` will do both
