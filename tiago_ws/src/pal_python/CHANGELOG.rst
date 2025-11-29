^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2021-08-25)
------------------
* Merge branch 'python3' into 'ferrum-devel'
  Python3 support
  See merge request common/pal_python!10
* Package format 3 and added python future dependency
* futurize'd python code
* Contributors: Jordan Palacios

0.9.18 (2021-02-01)
-------------------
* Fix param check for simulation
  Somehow it is not working, at least in ferrum
* Contributors: Victor Lopez

0.9.17 (2020-07-13)
-------------------
* Fixed which fd are closed
* Contributors: Jordan Palacios

0.9.16 (2020-06-25)
-------------------
* Merge branch 'shell_wait' into 'dubnium-devel'
  Shell wait
  See merge request common/pal_python!8
* Close fd opened by ourselves only
* Added wait function
* pep8
* Contributors: Jordan Palacios, jordanpalacios

0.9.15 (2019-03-04)
-------------------
* Merge branch 'subprocess_streaming' into 'dubnium-devel'
  added subprocess output streaming methods
  See merge request common/pal_python!7
* added subprocess output streaming methods
* Contributors: Sai Kishor Kothakota, Victor Lopez

0.9.14 (2017-07-12)
-------------------
* Add pal test utils
* Contributors: Victor Lopez

0.9.13 (2017-07-11)
-------------------
* Fix deps and maintainer
* Contributors: Victor Lopez

0.9.12 (2017-07-11)
-------------------
* Add ros_image_utils
* Contributors: Victor Lopez

0.9.11 (2017-03-30)
-------------------
* Reem Led client will use all leds
* Contributors: Victor Lopez

0.9.10 (2017-03-20)
-------------------
* Check if cmd is finished after sleeping
  Happens usually with rosbags
* Contributors: Victor Lopez

0.9.9 (2016-11-30)
------------------
* Adapt pal_hci to new led manager interface
* replace pal_startup with pal_startup_base
* Contributors: Jeremie Deray, Victor Lopez

0.9.8 (2016-02-29)
------------------
* Add configurable stdin, stdout, stderr
* Contributors: Victor Lopez

0.9.7 (2016-02-18)
------------------
* Add shell_cmd from remote_shell
* Contributors: Victor Lopez

0.9.6 (2015-09-16)
------------------
* Unhardcode Robot Folders
  I'm concerned that maybe someone could rely on the default behaviour (which I believe is wrong) of using `reem_maps` in a robot not called `reem`.
* Contributors: Sam Pfeiffer

0.9.5 (2015-04-10)
------------------
* Fix the path of start/stop files in desktop machines with pal_startup installed from our debians
* Contributors: Sammy Pfeiffer, Victor Lopez
* Changed maintainer to Sammy Pfeiffer

0.9.4 (2015-01-16)
------------------
* Add surveillance robot
* Update launch path (for pal_startup)
* Contributors: Enrique Fernandez, enriquefernandez

0.9.3 (2014-11-24)
------------------
* Add note on socket consumption of is_node_running
* Contributors: Enrique Fernandez

0.9.2 (2014-11-14)
------------------
* Allows to pass robot name to get maps path
* Sets maps path to $HOME/.pal/<robot>_maps
* Contributors: Enrique Fernandez

0.9.1 (2014-09-09)
------------------
* Initial release
* Contributors: Bence Magyar, Enrique Fernández Perdomo, Paul Mathieu, Siegfried-A. Gevatter Pujals, Víctor López
