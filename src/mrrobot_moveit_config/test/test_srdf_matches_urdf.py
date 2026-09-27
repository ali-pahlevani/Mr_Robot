"""Every joint and link the SRDF names must exist in the URDF, and each
planning group's chain must be one the URDF can actually form."""

import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRDF = os.path.join(HERE, "..", "config", "mrRobot.srdf")


@pytest.fixture(scope="module")
def urdf():
    from ament_index_python.packages import get_package_share_directory
    xacro = os.path.join(get_package_share_directory("mrrobot_description"),
                         "urdf", "mrRobot.urdf.xacro")
    text = subprocess.check_output(["xacro", xacro, "use_webots:=false",
                                    "use_ros2_control:=false"])
    return ET.fromstring(text)


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(SRDF).getroot()


def test_named_joints_and_links_exist(urdf, srdf):
    joints = {j.get("name") for j in urdf.iter("joint")}
    links = {l.get("name") for l in urdf.iter("link")}
    for el in srdf.iter():
        if el.tag in ("joint",) and el.get("name"):
            assert el.get("name") in joints, el.get("name")
        if el.tag == "link":
            assert el.get("name") in links, el.get("name")
        for attr in ("link1", "link2", "parent_link", "child_link", "tip_link"):
            if el.get(attr) and attr != "base_link":
                assert el.get(attr) in links, f"{el.tag} {attr}={el.get(attr)}"
        if el.tag == "chain":
            assert el.get("base_link") in links


def test_chains_are_connected(urdf, srdf):
    parent_of = {j.find("child").get("link"): j.find("parent").get("link")
                 for j in urdf.iter("joint")}
    for chain in srdf.iter("chain"):
        link, base = chain.get("tip_link"), chain.get("base_link")
        seen = [link]
        while link != base:
            assert link in parent_of, f"chain from {chain.get('tip_link')} never reaches {base}: {seen}"
            link = parent_of[link]
            seen.append(link)


def test_group_states_use_group_joints(urdf, srdf):
    parent_of = {j.find("child").get("link"): (j.find("parent").get("link"), j.get("name"), j.get("type"))
                 for j in urdf.iter("joint")}

    def chain_joints(base, tip):
        out, link = [], tip
        while link != base:
            parent, name, jtype = parent_of[link]
            if jtype != "fixed":
                out.append(name)
            link = parent
        return set(out)

    groups = {}
    for g in srdf.iter("group"):
        chain = g.find("chain")
        if chain is not None:
            groups[g.get("name")] = chain_joints(chain.get("base_link"), chain.get("tip_link"))
        else:
            groups[g.get("name")] = {j.get("name") for j in g.findall("joint") + g.findall("passive_joint")}
    for state in srdf.iter("group_state"):
        names = {j.get("name") for j in state.findall("joint")}
        assert names == groups[state.get("group")], \
            f"{state.get('name')}/{state.get('group')}: {names ^ groups[state.get('group')]}"


def test_lift_groups_add_exactly_the_lift_joint(urdf, srdf):
    """The *_arm_lift chains are the arm chains plus lift_joint."""
    parent_of = {j.find("child").get("link"): (j.find("parent").get("link"), j.get("name"), j.get("type"))
                 for j in urdf.iter("joint")}

    def chain_joints(base, tip):
        out, link = set(), tip
        while link != base:
            parent, name, jtype = parent_of[link]
            if jtype != "fixed":
                out.add(name)
            link = parent
        return out

    chains = {g.get("name"): g.find("chain") for g in srdf.iter("group") if g.find("chain") is not None}
    for side in ("left", "right"):
        arm = chain_joints(chains[f"{side}_arm"].get("base_link"), chains[f"{side}_arm"].get("tip_link"))
        full = chain_joints(chains[f"{side}_arm_lift"].get("base_link"), chains[f"{side}_arm_lift"].get("tip_link"))
        assert full - arm == {"lift_joint"}
        assert len(arm) == 7
