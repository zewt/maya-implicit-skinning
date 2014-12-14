VCG lib core is composed only with headers.
So no need to worry about compiling the lib.
Just include the file you need inside the project.

Note: Not all components of vcglib are here because
we don't need all of them and many aren't compile free headers.
also eigenlib is duplicated here because I'm not sure implicit_skinning and vcglib
are actually using the same version.

If you add the missing modules of the vcg library:
    - be sure to fetch the same revision (or ovewrite everything in this folder).
    - check if anything need to be compiled.