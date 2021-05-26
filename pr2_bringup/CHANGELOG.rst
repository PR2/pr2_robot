^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.32 (2021-05-26)
-------------------
* Merge pull request `#268 <https://github.com/pr2/pr2_robot/issues/268>`_ from k-okada/fix_for_noetic
* run 2to3 -w -fexcept .
* run 2to3 -w -fprint .
* Contributors: Kei Okada

1.6.31 (2020-04-14)
-------------------
* pr2_bringup: test launch files (`#255 <https://github.com/pr2/pr2_robot/issues/255>`_)

  * Disable because pr2.launch needs to resolve c1/c2 hostname

* rebased version of https://github.com/PR2/pr2_robot/pull/256 (`#263 <https://github.com/pr2/pr2_robot/issues/263>`_)

  * Adapt monitoring to upgraded PR2s:
    - no more IPMI
    - new number of CPU cores
    - re-add monitoring of c2

* calibrate_pr2: intercept and print exception (`#248 <https://github.com/pr2/pr2_robot/issues/248>`_)
  * This was silently ignored before.
  * White space cleanup

* Param names changed between urg_node and hokuyo_node (`#257 <https://github.com/pr2/pr2_robot/issues/257>`_)

  * The new urg_node package is used for the laser scanners in kinetic, but the parameter names are still the one of the hokuyo_node package.

* Contributors: Yuki Furuta, Matthieu Herrb, Michael Goerner, Yannick Jonetzko

1.6.30 (2018-04-23)
-------------------

1.6.29 (2018-04-22)
-------------------
* commented out tests that seem to require PR2 to function properly (timing out jenkins build)
* Contributors: David Feil-Seifer

1.6.28 (2018-04-21)
-------------------
* made sure tests only run if CATKIN_ENABLE_TESTING is set
* Contributors: David Feil-Seifer

1.6.27 (2018-04-20)
-------------------
* updated hokuyo node dependencies to urg_ndoe
* Contributors: David Feil-Seifer

1.6.26 (2018-03-19)
-------------------

1.6.25 (2018-03-19)
-------------------
* updated packages for new maintainer
* updated changelogs
* Contributors: David Feil-Seifer

1.6.7 (2015-02-11)
------------------
* Updated mainpage.dox
* Contributors: TheDash
