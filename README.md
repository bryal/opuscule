# Opuscule

This is (going to be) a pure-Rust implementation of the Opus audio decoder,
targeting `no_std` environments like microcontrollers.


## Limitations

This is a **decoder only**.


## Development status

This is my second (or maybe third) attempt at this.
Haven't even gotten started here yet -
it's just us and the wide open road.

The plan is to convert the reference decoder from C to Rust piece by piece,
passing all tests every step along the way.
Then, when we have a correct and working Rust decoder in place,
we'll clean up any remaining C traces,
and finally start looking into "carcinizing" the code
and maybe optimize it if we can manage.
