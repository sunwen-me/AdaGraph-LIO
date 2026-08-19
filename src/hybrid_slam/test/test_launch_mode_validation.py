#!/usr/bin/env python3
"""Launch-argument validation tests for the Mid-360 SLAM launch file.

The launch file is the only place that rejects inconsistent mapping and
localization argument combinations before the node starts, so the rules are
exercised directly here instead of through a full launch.
"""

import importlib.util
import os
import unittest

from launch import LaunchContext
from launch.utilities import perform_substitutions

_LAUNCH_FILE = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'launch', 'PointLIO_mid360.py',
    )
)

# An existing absolute path is all the prior-map check requires; validation
# never opens or parses the file.
_EXISTING_ABSOLUTE_FILE = os.path.abspath(__file__)

_DEFAULTS = {
    'mode': 'mapping',
    'use_prior_map': 'false',
    'prior_map_file': '',
    'save_nav2_map': 'false',
    'nav2_map_yaml': '',
    'enable_backend_localization_factor': 'false',
    'save_map': 'false',
    'save_map_dir': 'map',
}


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        'point_lio_mid360_launch', _LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_LAUNCH = _load_launch_module()


def _validate(**overrides):
    context = LaunchContext()
    values = dict(_DEFAULTS)
    values.update(overrides)
    for key, value in values.items():
        context.launch_configurations[key] = value
    return _LAUNCH._validate_slam_mode(context)


class TestSlamModeValidation(unittest.TestCase):
    def test_mapping_defaults_are_accepted(self):
        self.assertTrue(_validate())

    def test_unknown_mode_is_rejected(self):
        with self.assertRaises(RuntimeError):
            _validate(mode='navigation')

    def test_non_lowercase_mode_is_rejected(self):
        # The occupancy-source substitution compares the raw string, so a
        # value that only differs in case must not reach the node.
        with self.assertRaises(RuntimeError):
            _validate(mode='Mapping')

    def test_localization_requires_prior_map_enabled(self):
        with self.assertRaises(RuntimeError):
            _validate(mode='localization',
                      prior_map_file=_EXISTING_ABSOLUTE_FILE)

    def test_localization_requires_absolute_prior_map(self):
        with self.assertRaises(RuntimeError):
            _validate(mode='localization', use_prior_map='true',
                      prior_map_file='relative/site.pcd')

    def test_localization_requires_existing_prior_map(self):
        with self.assertRaises(RuntimeError):
            _validate(mode='localization', use_prior_map='true',
                      prior_map_file='/nonexistent/site.pcd')

    def test_localization_accepts_existing_prior_map(self):
        self.assertTrue(_validate(mode='localization', use_prior_map='true',
                                  prior_map_file=_EXISTING_ABSOLUTE_FILE))

    def test_localization_rejects_map_saving(self):
        # Saving during localization would write the live session map, not the
        # prior the run was localizing against.
        with self.assertRaises(RuntimeError):
            _validate(mode='localization', use_prior_map='true',
                      prior_map_file=_EXISTING_ABSOLUTE_FILE,
                      save_map='true')

    def test_localization_factor_requires_prior_map(self):
        with self.assertRaises(RuntimeError):
            _validate(enable_backend_localization_factor='true')

    def test_nav2_artifact_requires_mapping_mode(self):
        with self.assertRaises(RuntimeError):
            _validate(mode='localization', use_prior_map='true',
                      prior_map_file=_EXISTING_ABSOLUTE_FILE,
                      save_nav2_map='true',
                      nav2_map_yaml='/tmp/site.yaml')

    def test_nav2_artifact_requires_absolute_yaml(self):
        with self.assertRaises(RuntimeError):
            _validate(save_nav2_map='true', nav2_map_yaml='site.yaml')

    def test_nav2_artifact_requires_yaml_extension(self):
        with self.assertRaises(RuntimeError):
            _validate(save_nav2_map='true', nav2_map_yaml='/tmp/site.pgm')

    def test_nav2_artifact_accepts_absolute_yaml(self):
        self.assertTrue(
            _validate(save_nav2_map='true', nav2_map_yaml='/tmp/site.yaml'))

    def test_map_saving_reports_resolved_directory(self):
        messages = _validate(save_map='true', save_map_dir='map')
        # LogInfo stores its message as a substitution list, so the reported
        # text only exists once the substitutions are performed. These are
        # plain text substitutions, so an empty context resolves them.
        rendered = ' '.join(
            perform_substitutions(LaunchContext(), message.msg)
            for message in messages
        )
        self.assertIn(os.path.abspath('map'), rendered)


if __name__ == '__main__':
    unittest.main()
