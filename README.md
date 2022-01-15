# uvpackit

Modo Kit that includes a command for packing UVs, powered by UVPackmaster 2

The included command can be executed with `uvp.pack` which will open a dialog with available options.

## Installing

Download lpk from releases. Drag and drop into your Modo viewport. If you're upgrading, delete previous version.

## Setting up development environment

Download and install CMake, Visual Studio 2019, Modos LXSDK and the SDK for [UVPackmaster](https://glukoz.gumroad.com/l/uvpackmaster2-sdk-std) 

Assuming you opted for running CMake GUI, 

__Where is the source code__, Browse to root of this kit. 

__Where to build the binaries__, I've worked using a subfolder "build".

Now running __Configure__ should show you some values likely set red. Here you will want to set LXSDK_PATH by browsing to where you downloaded the Modo SDK.

UVPackmaster have an installer that will default to installing the SDK at a given location, if you downloaded a later version or installed to another location you will have to change the following two values as well.

- UVP_INCLUDE
- UVP_LIBRARY

Once configuration is done, you should be able to generate a project.

If you cloned this repo into your kit folder, everything should be ready for your next Modo session.

## Packaging the LPK

To create the LPK and distribute the plug-in. Create a zip with the dynamic libraries, configs and index.xml and icons. Make sure to update the index.xml with the intended contents for the kit.

Change the extension from zip to lpk and this should allow users to drag-drop the file into their Modo viewport and have the kit installed to their user contents.