compile g7000ram-1.5\example\hello.a48
=========================================

Use compiler aswcurr. Put C:\aswcurr\bin in your PATH environment

copmile with

asw.exe example\hello.a48 -cpu 8048 -i src

The -i src is the path to g7000.h header file

Use p2bin.exe to convert to binary format.
