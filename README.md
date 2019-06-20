# BodyTrackingProjectorInteraction

# Dependencies

   This relies on the `dynamixel_motor` and `body_tracker_messages` repositories

# Installation Instructions / Prerequisites

## Follow instructions on Nuitrack website
  - http://download.3divi.com/Nuitrack/doc/Installation_page.html

  - Clone this project into your catkin workspace

  - Clone the `dynamixel_motor` and `body_tracker_messages` into your catkin workspace
  
  - Remove OpenNI - it conflicts with the version supplied by Nuitrack!
    -   `sudo apt-get purge --auto-remove openni-utils`

  - Download BOTH the nuitrack Linux drivers and the Nuitrack SDK

  - Install Nuitrack Linux drivers:
    -   `sudo dpkg -i nuitrack-ubuntu-amd64.deb`
    -   `sudo reboot`
    -   confirm environment variables set correctly:
        - `echo $NUITRACK_HOME`    (should be /usr/etc/nuitrack)
        - `echo $LD_LIBRARY_PATH`  (should include /usr/local/lib/nuitrack)

  - Install Nuitrack SDK (NuitrackSDK.zip)
    - Download from: http://download.3divi.com/Nuitrack/
    - `mkdir ~/sdk/NuitrackSDK`
    - `cp NuitrackSDK.zip ~/NuitrackSDK`
    - extract ZIP archive with ubuntu Archive Manager (double click the zip file)
    - delete the zip file

  - Edit CMakeLists.txt if you installed the SDK to a different location:
    `set(NUITRACK_SDK_PATH /home/system/sdk/NuitrackSDK)`

