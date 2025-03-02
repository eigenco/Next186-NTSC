del Next186.lst
del Next186.map
del Next186.obj
del *.ROM
ml /AT /c /Fl Next186.asm
link16 /TINY Next186,Next186.com,,,,
ren Next186.com Next186.ROM
del Next186.lst
del Next186.map
del Next186.obj
pause