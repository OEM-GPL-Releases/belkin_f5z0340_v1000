In an attempt to fully compliant with the Open Source License terms and agreements, we're releasing open source package used for our products.

Please install toolchain(wemo_toolchain/README) before building the image.

Here's the brief description how one can build the same image but not signed.

1. untar and ungzip the downloaded source package.
   i.e. tar -xzvf wemo_gpl_sns_package-9041.tar.gz

2. Change directory into wemo_gpl_sns_package-9041.

3. Use the following command to start the build process.
   make sns

4. The final output files will be placed under ./output directory.
firmware_2.00.9041.OWRT.DVT_SNS.bin : complete firmware image
WeMo_WW_2.00.9041.OWRT.DVT_SNS.bin : complete firmware image with belkin header

