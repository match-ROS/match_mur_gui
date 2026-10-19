"""Configure direct DDS discovery before any of the GUI's ROS contexts start."""

import hashlib
import ipaddress
import os
from pathlib import Path
import socket
import tempfile
import xml.etree.ElementTree as ET


def resolve_ipv4(host):
    return sorted({entry[4][0] for entry in socket.getaddrinfo(
        host, None, family=socket.AF_INET, type=socket.SOCK_DGRAM)})


def ssh_reachable(address):
    try:
        with socket.create_connection((address, 22), timeout=0.25):
            return True
    except OSError:
        return False


def configure_ros_discovery(hosts, directory, *, environ=None, resolver=None, reachable=None):
    """Add explicit peer ports while preserving operator-supplied DDS settings.

    A plain ROS_STATIC_PEERS address does not discover all participants in our
    MuR hardware launch when multicast is unavailable. Probe up to 64 participant
    discovery ports per peer, using the same domain as the GUI and its children.
    """
    env = os.environ if environ is None else environ
    resolver = resolve_ipv4 if resolver is None else resolver
    reachable = ssh_reachable if reachable is None else reachable
    env.setdefault('ROS_DOMAIN_ID', '62')
    if (env.get('ROS_LOCALHOST_ONLY') == '1'
            or env.get('ROS_AUTOMATIC_DISCOVERY_RANGE', '').upper() in ('LOCALHOST', 'OFF')):
        return 'DDS: local-only discovery explicitly configured; remote peers not added'
    env.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    if env.get('ROS_DISCOVERY_SERVER'):
        return 'DDS: using the configured discovery server'
    rmw = env.get('RMW_IMPLEMENTATION', '')
    if rmw and rmw not in ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp'):
        return f'DDS: preserving configured middleware {rmw}'

    profile_keys = ('FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE')
    for key in profile_keys:
        profile = env.get(key)
        if profile and Path(profile).is_file():
            return f'DDS: using configured profile {profile}'
    for key in profile_keys:
        env.pop(key, None)

    domain = int(env['ROS_DOMAIN_ID'])
    if not 0 <= domain <= 232:
        raise ValueError('ROS_DOMAIN_ID must be between 0 and 232')
    explicit_hosts = [host.strip() for host in env.get('ROS_STATIC_PEERS', '').split(';')
                      if host.strip()]
    peer_hosts = list(dict.fromkeys([*hosts, *explicit_hosts]))
    addresses = set()
    unresolved = []
    offline = []
    for host in peer_hosts:
        if not host.strip():
            continue
        try:
            for address in resolver(host.strip()):
                address = str(ipaddress.IPv4Address(address))
                # Offline robot addresses can stall Fast DDS on repeated ARP
                # attempts for every participant port. Connect itself also
                # requires SSH; explicit extra DDS peers need not offer SSH.
                if address in addresses or host.strip() in explicit_hosts or reachable(address):
                    addresses.add(address)
                else:
                    offline.append(host)
        except (OSError, ValueError):
            unresolved.append(host)
    if not addresses:
        return 'DDS: no reachable MuR peers; check the network and restart the GUI after connecting'

    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    participant = ET.SubElement(root, 'participant',
                                profile_name='mur_gui_direct_peers', is_default_profile='true')
    peers = ET.SubElement(ET.SubElement(ET.SubElement(participant, 'rtps'), 'builtin'),
                          'initialPeersList')
    ports = [7400 + 250 * domain + 10 + 2 * index for index in range(64)]
    for address in sorted(addresses):
        for port in ports:
            if port > 65535:
                break
            locator = ET.SubElement(ET.SubElement(peers, 'locator'), 'udpv4')
            ET.SubElement(locator, 'address').text = address
            ET.SubElement(locator, 'port').text = str(port)
    content = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    digest = hashlib.sha256(content).hexdigest()[:12]
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / f'peers_domain{domain}_{digest}.xml'
    if not destination.is_file() or destination.read_bytes() != content:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as temporary:
            temporary.write(content)
            pending = Path(temporary.name)
        try:
            pending.replace(destination)
        finally:
            pending.unlink(missing_ok=True)
    # Also replace stale /tmp profile references inherited from an older launch.
    for key in profile_keys:
        env[key] = str(destination)
    detail = f'DDS: direct peers {", ".join(sorted(addresses))}, domain {domain}'
    if unresolved:
        detail += f'; unresolved hosts: {", ".join(unresolved)}'
    if offline:
        detail += f'; offline robots skipped: {", ".join(dict.fromkeys(offline))}'
    return detail
