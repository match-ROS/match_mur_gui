"""Normal GUI starts discover all MuR participants without a temporary wrapper."""

from pathlib import Path
import socket
import xml.etree.ElementTree as ET

import pytest

from match_mur_gui.ros_discovery import configure_ros_discovery


def endpoints(profile):
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(profile).getroot()
    return [(node.find('d:address', ns).text, int(node.find('d:port', ns).text))
            for node in root.findall('.//d:locator/d:udpv4', ns)]


def test_clean_gui_environment_gets_all_participant_ports_and_default_domain(tmp_path):
    env = {}
    message = configure_ros_discovery(
        ['mur620c'], tmp_path, environ=env, reachable=lambda address: True, resolver=lambda host: ['10.145.8.41'])
    assert env['ROS_DOMAIN_ID'] == '62'
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SUBNET'
    assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == env['FASTDDS_DEFAULT_PROFILES_FILE']
    assert endpoints(env['FASTRTPS_DEFAULT_PROFILES_FILE']) == [
        ('10.145.8.41', port) for port in range(22910, 23038, 2)]
    assert '10.145.8.41' in message and 'domain 62' in message
    assert Path(env['FASTRTPS_DEFAULT_PROFILES_FILE']).parent == tmp_path


def test_missing_old_tmp_profile_is_replaced_and_extra_peers_are_preserved(tmp_path):
    env = {'FASTRTPS_DEFAULT_PROFILES_FILE': str(tmp_path / 'removed.xml'),
           'ROS_STATIC_PEERS': 'mur620c;extra'}
    def resolve(host):
        if host == 'mur620a':
            raise socket.gaierror('not on this network')
        return ['10.145.8.42' if host == 'extra' else '10.145.8.41']
    message = configure_ros_discovery(['mur620a', 'mur620c'], tmp_path,
                                       environ=env, reachable=lambda address: True, resolver=resolve)
    pairs = endpoints(env['FASTRTPS_DEFAULT_PROFILES_FILE'])
    assert len(pairs) == 128
    assert {ip for ip, _ in pairs} == {'10.145.8.41', '10.145.8.42'}
    assert 'unresolved hosts: mur620a' in message
    assert env['ROS_STATIC_PEERS'] == 'mur620c;extra'


@pytest.mark.parametrize('domain', [0, 73, 232])
def test_custom_domain_changes_ports_and_stays_within_udp_range(tmp_path, domain):
    env = {'ROS_DOMAIN_ID': str(domain)}
    configure_ros_discovery(['mur620c'], tmp_path, environ=env, reachable=lambda address: True,
                            resolver=lambda host: ['10.145.8.41'])
    ports = [port for _, port in endpoints(env['FASTRTPS_DEFAULT_PROFILES_FILE'])]
    assert ports[0] == 7400 + 250 * domain + 10
    assert all(0 < port <= 65535 for port in ports)


@pytest.mark.parametrize('key', ['FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE'])
def test_existing_operator_profile_is_not_overwritten(tmp_path, key):
    profile = tmp_path / 'operator.xml'
    profile.write_text('custom DDS settings')
    env = {key: str(profile)}
    message = configure_ros_discovery(['mur620c'], tmp_path / 'unused', environ=env, reachable=lambda address: True,
                                       resolver=lambda host: pytest.fail('must not resolve'))
    assert env[key] == str(profile)
    assert profile.read_text() == 'custom DDS settings'
    assert 'using configured profile' in message
    assert not (tmp_path / 'unused').exists()


@pytest.mark.parametrize('override', [
    {'ROS_DISCOVERY_SERVER': 'server:11811'},
    {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'},
    {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
    {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF'},
    {'ROS_LOCALHOST_ONLY': '1'},
])
def test_explicit_network_modes_are_respected(tmp_path, override):
    env = dict(override)
    configure_ros_discovery(['mur620c'], tmp_path, environ=env, reachable=lambda address: True,
                            resolver=lambda host: pytest.fail('must not resolve'))
    assert all(env[key] == value for key, value in override.items())
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in env


def test_profile_is_repeatable_without_rewriting_an_active_file(tmp_path):
    def run():
        env = {}
        configure_ros_discovery(['mur620c'], tmp_path, environ=env, reachable=lambda address: True,
                                resolver=lambda host: ['10.145.8.41'])
        return Path(env['FASTRTPS_DEFAULT_PROFILES_FILE'])
    first = run()
    modified = first.stat().st_mtime_ns
    assert run() == first
    assert first.stat().st_mtime_ns == modified


def test_offline_network_is_reported_without_blocking_gui_start(tmp_path):
    def offline(host):
        raise socket.gaierror('offline')
    env = {}
    message = configure_ros_discovery(['mur620c'], tmp_path, environ=env, reachable=lambda address: True, resolver=offline)
    assert 'no reachable MuR peers' in message
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in env


def test_offline_robots_are_skipped_without_losing_online_peers(tmp_path):
    env = {}
    message = configure_ros_discovery(
        ['mur620a', 'mur620c'], tmp_path, environ=env,
        resolver=lambda host: ['10.145.8.41' if host == 'mur620c' else '10.145.8.60'],
        reachable=lambda address: address == '10.145.8.41')
    assert {ip for ip, _ in endpoints(env['FASTRTPS_DEFAULT_PROFILES_FILE'])} == {'10.145.8.41'}
    assert 'offline robots skipped: mur620a' in message


def test_unreachable_peers_do_not_keep_a_missing_profile_reference(tmp_path):
    env = {'FASTRTPS_DEFAULT_PROFILES_FILE': str(tmp_path / 'deleted.xml')}
    message = configure_ros_discovery(
        ['mur620c'], tmp_path, environ=env, resolver=lambda host: ['10.145.8.41'],
        reachable=lambda address: False)
    assert 'no reachable MuR peers' in message
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in env
