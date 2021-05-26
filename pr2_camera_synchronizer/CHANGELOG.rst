^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_camera_synchronizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.32 (2021-05-26)
-------------------
* Merge pull request `#268 <https://github.com/pr2/pr2_robot/issues/268>`_ from k-okada/fix_for_noetic
* async is keyword in Python3.7
* python3 did not take tuple for lambda : https://stackoverflow.com/questions/11328312/python-lambda-does-not-accept-tuple-argument
* run 2to3 -w -fexcept .
* run 2to3 -w -fprint .
* Contributors: Kei Okada

1.6.31 (2020-04-14)
-------------------

1.6.30 (2018-04-23)
-------------------
* removed more tests for jenkins build
* Contributors: David Feil-Seifer

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
