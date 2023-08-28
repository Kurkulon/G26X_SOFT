
##################################################################################################

!ifndef version
version = Release
!endif

##################################################################################################

curdir = $+ $(%cdrive):$(%cwd) $-
cppdir = cpp
objdir = $(version)

##################################################################################################

.BEFORE
	@if NOT EXIST $(objdir) @md $(objdir) >nul

##################################################################################################

make : .SYMBOLIC
	@call wmake -h -f mkmodules cppdir=$(cppdir)\ objdir=$(objdir)\
!ifneq version Win32_Debug
	@call wmake -h -f makedep version=$(version) cppdir=$(cppdir) objdir=$(objdir)
!endif
	@call wmake $(__MAKEOPTS__) -f makeobj version=$(version) buildtool=$(buildtool) cppdir=$(cppdir) objdir=$(objdir)


##################################################################################################

