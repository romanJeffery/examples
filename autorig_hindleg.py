"""

Created on 30 April 2019

@author: Roman Jeffery

"""

import maya.cmds as mc
import maya.api.OpenMaya as om
import rj_toolset.rj_constraint as rjc


class Hindleg(object):

    def __init__(self, side, instance_num):

        self.instance_num = instance_num
        self.side = side
        self.name = 'hindleg'
        self.suffix = 'grp'
        self.moduleGrp = 'module_grp'
        self.main_grps = ["mod", "jnts", "FKcontrol", "IKcontrol", "noTransform", "curves"]
        self.parent_order = ["pelvis", "hip", "knee", "foot", "ball", "toe", "toeEnd"]
        self.guides = self._getNumOfInstances()
        self.mod_dict = {}
        self.fk_joints = {}
        self.ik_joints = {}
        self.ikSolvers = {}
        self.ik_controllers = {}
        self.fk_controllers = {}
        self.final_joints = {}
        self.twist_joints = {}
        self.joint_dict = {}
        self.nurb_drivers = {}
        self.curves = {}
        self.nurbs = {}
        self.clusters = {}
        self.misc = {}

    def _getNumOfInstances(self):
        """Get all guides in the scene associated with this specific module."""

        numb_dict = {}
        numb = mc.ls("*.hindleg")
        for i in numb:

            item = i.split(".")[0]
            pos = mc.xform(item, q=1, ws=1, t=1)
            mat = mc.xform(item, q=1, ws=1, matrix=1)
            numb_dict[item.split("_")[2]] = [item, pos, mat]

        return numb_dict

    def _createStructure(self):
        """Create our module hierarchy including all main groups to parent items under."""

        for i, x in enumerate(self.main_grps):

            if i == 0:
                self.mod_dict[x] = "{}_{}_{}_{}".format(self.side, self.name, x, self.instance_num)
                mc.group(n=self.mod_dict[x], empty=True, parent=self.moduleGrp)
            else:
                self.mod_dict[x] = "{}_{}_{}_{}".format(self.side, self.name, x,  self.suffix, self.instance_num)
                mc.group(n=self.mod_dict[x], empty=True, parent=self.mod_dict["mod"])

        mc.hide(self.mod_dict["noTransform"], self.mod_dict["curves"])

        self._turnOffInheritTransform(self.mod_dict["noTransform"])

    def _turnOffInheritTransform(self, item):
        """Turn off inherit transform on node."""

        mc.setAttr(item + ".inheritsTransform", 0)

    def _createIkFootAttrs(self, ctl):
        """Create attrs on IK foot control."""

        mc.addAttr(ctl, at="enum", ln="_________", k=1, en="SETTINGS")
        mc.addAttr(ctl, ln="ikFk", at="float", k=1, min=0, max=1, dv=1)
        mc.addAttr(ctl, ln="autoHock", at="float", k=1, min=0, max=1, dv=1)
        mc.addAttr(ctl, ln="stretchHock", at="float", k=1, dv=0)
        mc.addAttr(ctl, ln="stretchy", at="float", k=1, min=0, max=1, dv=1)
        mc.addAttr(ctl, at="enum", ln="__________", k=1, en="POSE")
        mc.addAttr(ctl, ln="hockRoll", at="float", k=1, dv=0)
        mc.addAttr(ctl, ln="toeRoll", at="float", k=1, dv=0)
        mc.addAttr(ctl, ln="heelRoll", at="float", k=1, dv=0)
        mc.addAttr(ctl, ln="toeSwivel", at="float", k=1, dv=0)
        mc.addAttr(ctl, ln="heelSwivel", at="float", k=1, dv=0)
        mc.addAttr(ctl, ln="footBank", at="float", k=1, dv=0)

    def _makeJointChains(self, chain, joint_dict):
        """Create the joint chains by referencing the guides in the scene."""

        prev_jnt = None
        joint_ls = []
        if not joint_dict:
            joint_dict = {}

        for i in self.guides.items():
            mc.select(cl=1)
            jnt = mc.joint(n=i[1][0].replace("_guide", chain + "_jnt"))
            mc.xform(jnt, ws=1, t=i[1][1])

            split_name = i[1][0].replace("guide", "jnt").split("_")[2]
            split_jnt = jnt.split("_")[2]

            self.joint_dict[split_jnt] = jnt
            joint_dict[split_name] = jnt

            joint_ls.append(jnt)

        for i, x in enumerate(self.parent_order):

            if i == 0:
                mc.parent(joint_dict[x], self.mod_dict["jnts"])
                prev_jnt = joint_dict[x]
            else:
                mc.parent(joint_dict[x], prev_jnt)
                prev_jnt = joint_dict[x]

        for i in joint_dict.values():
            split_jnt = i.split("_")[2]
            if "roll" in split_jnt:
                mc.parent(self.joint_dict[split_jnt], self.joint_dict["ball" + chain])

        for i in joint_ls:
            mc.joint(i, e=1, zso=1, oj='zxy')

        if not chain == "FINAL":
            mc.hide(joint_dict["pelvis"])
        else:
            mc.rename(self.joint_dict["footFINAL"], self.joint_dict["footFINAL"].replace("jnt", "bnd"))
            mc.rename(self.joint_dict["ballFINAL"], self.joint_dict["ballFINAL"].replace("jnt", "bnd"))
            mc.rename(self.joint_dict["toeFINAL"], self.joint_dict["toeFINAL"].replace("jnt", "bnd"))
            mc.rename(self.joint_dict["toeEndFINAL"], self.joint_dict["toeEndFINAL"].replace("jnt", "bnd"))

            self.joint_dict["footFINAL"] = self.joint_dict["footFINAL"].replace("jnt", "bnd")
            self.joint_dict["ballFINAL"] = self.joint_dict["ballFINAL"].replace("jnt", "bnd")
            self.joint_dict["toeFINAL"] = self.joint_dict["toeFINAL"].replace("jnt", "bnd")
            self.joint_dict["toeEndFINAL"] = self.joint_dict["toeEndFINAL"].replace("jnt", "bnd")

    def _createJoints(self):
        """Create all the joint chains necessary for this module."""

        self._makeJointChains("FK", self.fk_joints)
        self._makeJointChains("IK", self.ik_joints)
        self._makeJointChains("SPRING", self.ik_joints)
        self._makeJointChains("FINAL", self.final_joints)

    def _createMasterControl(self):
        """Create main hindleg control with attrs."""

        hindleg_main_ctl = mc.circle(n="{}_{}_ctl_{}".format(self.side, self.name, self.instance_num), r=1)[0]
        hindleg_offset_grp = mc.group(hindleg_main_ctl, n=hindleg_main_ctl.replace("ctl", "grp"))
        hindleg_zero_grp = mc.group(hindleg_offset_grp, n=hindleg_main_ctl.replace("ctl", "zero"),
                                   p=self.mod_dict["FKcontrol"])

        pos = mc.xform(self.joint_dict["footFINAL"], q=1, ws=1, t=1)
        mc.xform(hindleg_zero_grp, ws=1, t=pos)
        mc.xform(hindleg_offset_grp, t=[1, 0, 0])

        self._createIkFootAttrs(hindleg_main_ctl)

        self.ik_controllers[hindleg_main_ctl.split("_")[1]] = hindleg_main_ctl

        rjc.constrain_object(self.joint_dict["footFINAL"], hindleg_zero_grp, "parent", 1)

    def _getCurvePositions(self, start, end):
        """Get positions along axis to create a curve from."""

        thigh_curve_points = []
        pos1 = self.guides[start][1]
        pos2 = self.guides[end][1]
        vec1 = [(pos1[0]+pos2[0]) / 2, (pos1[1]+pos2[1]) / 2, (pos1[2]+pos2[2]) / 2]
        pos3 = [(pos1[0]+vec1[0]) / 2, (pos1[1]+vec1[1]) / 2, (pos1[2]+vec1[2]) / 2]
        pos4 = [(pos2[0] + vec1[0]) / 2, (pos2[1] + vec1[1]) / 2, (pos2[2] + vec1[2]) / 2]

        for pos in [pos1, pos3, pos4, pos2]:
            thigh_curve_points.append(pos)

        return thigh_curve_points

    def _createNurbs(self):
        """Create the nurbs curves that will act as our bendy/twist curves."""

        for crv in [["hip", "knee"], ["knee", "foot"]]:

            limb_points = self._getCurvePositions(crv[0], crv[1])
            limb_crv = mc.curve(n=self.guides[crv[0]][0].replace("guide", "crv"), p=limb_points, d=3)
            limb_crv_offset = mc.offsetCurve(limb_crv, d=.3, ugn=0)
            limb_crv_inv = mc.offsetCurve(limb_crv, d=-.3, ugn=0)
            limb_loft = mc.loft(limb_crv_inv, limb_crv_offset, n=self.guides[crv[0]][0].replace("guide", "nrb"), ch=0)
            mc.delete(limb_crv, limb_crv_offset, limb_crv_inv)
            self.nurbs[crv[0]] = limb_loft[0]

            mc.parent(limb_loft, self.mod_dict["noTransform"])

    def _createTwistJoints(self):
        """Create and attach the joints along our created nurbs curves for hip and knee."""

        joints_num = 7
        uv_split = 1.0 / (joints_num - 1)
        prev_coord = 0

        twist_grp = mc.group(n="{}_{}_twistJoints_{}_{}".format(self.side, self.name, self.suffix, self.instance_num),
                             em=1, p=self.mod_dict["jnts"])

        for limb in [["upper", self.nurbs["hip"]], ["lower", self.nurbs["knee"]]]:
            for i, x in enumerate(range(joints_num)):

                mc.select(cl=1)
                twist_jnt = mc.joint(n="{}_{}_{}_twist{}_bnd_0".format(self.side, self.name, limb[0], i))
                plug = "normalizedNormal", "normalizedTangentV", "normalizedTangentU", "position"

                psi = mc.createNode("pointOnSurfaceInfo", n=twist_jnt.replace("bnd", "psi"))
                ffm = mc.createNode("fourByFourMatrix", n=twist_jnt.replace("bnd", "ffm"))
                dcm = mc.createNode("decomposeMatrix", n=twist_jnt.replace("bnd", "dcm"))

                if i == 0:
                    v_coord = 0
                    prev_coord = 0
                elif i == (joints_num - 1):
                    v_coord = 1
                else:
                    v_coord = prev_coord + uv_split
                    prev_coord = v_coord

                mc.setAttr(psi + ".parameterV", v_coord)
                mc.setAttr(psi + ".parameterU", .5)

                mc.connectAttr(limb[1] + ".worldSpace", psi + ".inputSurface")

                for num, plg in enumerate(plug):
                    mc.connectAttr("{}.{}X".format(psi, plg), ffm + ".in{}0".format(num))
                    mc.connectAttr("{}.{}Y".format(psi, plg), ffm + ".in{}1".format(num))
                    mc.connectAttr("{}.{}Z".format(psi, plg), ffm + ".in{}2".format(num))

                mc.connectAttr(ffm + ".output", dcm + ".inputMatrix")
                mc.connectAttr(dcm + ".outputTranslate", twist_jnt + ".t")
                mc.connectAttr(dcm + ".outputRotate", twist_jnt + ".r")

                mc.parent(twist_jnt, twist_grp)

                pos = mc.xform(twist_jnt, q=1, ws=1, matrix=1)
                self.twist_joints["{}twist{}".format(limb[0], i)] = [twist_jnt, pos]

        self._turnOffInheritTransform(twist_grp)

    def _createTwistJointDrivers(self):
        """Create driver joints to constrain the nurbs curve to."""

        dgrp = mc.group(n="{}_{}_nurbsDrivers_{}_{}".format(self.side, self.name, self.suffix, self.instance_num),
                        em=1, p=self.mod_dict["noTransform"])

        for limb in [["upper", self.nurbs["hip"]], ["lower", self.nurbs["knee"]]]:
            item = limb[1].split("_")[2]
            jnt_ls = []

            for i, x in enumerate(["0", "3", "6"]):
                mc.select(cl=1)
                jnt = mc.joint(n=limb[1].replace("_nrb", "Driver{}_jnt".format(i)))
                grp = mc.group(jnt, n=jnt.replace("jnt", "grp"))
                mc.xform(grp, ws=1, matrix=self.twist_joints["{}twist{}".format(limb[0], x)][1])
                self.clusters[item + str(i) + "jnt"] = jnt
                self.clusters[item + str(i)] = grp
                mc.parent(grp, dgrp)
                jnt_ls.append(jnt)

            self.nurb_drivers[limb[0]] = jnt_ls

    def _constrainNurbsDrivers(self):
        """Constrain nurbs driver joints. """

        rjc.constrain_object(self.joint_dict["hipFINAL"], self.clusters["hip0"], "parent", 1)

        rjc.constrain_object(self.joint_dict["kneeFINAL"], self.clusters["hip2"], "parent", 1)

        rjc.constrain_object(self.joint_dict["kneeFINAL"], self.clusters["knee0"], "parent", 1)

        rjc.constrain_object(self.joint_dict["footFINAL"], self.clusters["knee2"], "parent", 1)

    def _createBendControls(self):
        """Create bendy controls. """

        bendy_grp = mc.group(n="{}_{}_bendy_{}_{}".format(self.side, self.name, self.suffix, self.instance_num),
                             p=self.mod_dict["FKcontrol"], em=1)

        for limb in [["upper", self.nurbs["hip"]], ["lower", self.nurbs["knee"]]]:

            if limb[0] == "upper":
                cnstr = ["hip", "knee"]
            else:
                cnstr = ["knee", "foot"]

            ctl = mc.circle(n=limb[1].replace("_nrb", "Bendy_ctl"), r=2)[0]
            grp = mc.group(ctl, n=ctl.replace("ctl", "grp"))
            rjc.constrain_object([self.clusters[cnstr[0] + "0jnt"],
                                  self.clusters[cnstr[0] + "2jnt"]],
                                 grp, "point", 0)

            mc.parent(grp, bendy_grp)

            mc.aimConstraint(self.clusters[cnstr[0] + "2jnt"], grp, aimVector=[0, 0, 1],
                             upVector=[0, 1, 0], worldUpType="objectrotation",
                             worldUpVector=[0, 0, -1], worldUpObject=self.clusters[cnstr[0] + "0jnt"],
                             n=grp.replace("grp", "aic"))

            rjc.constrain_object(ctl, self.clusters[cnstr[0] + "1"],
                                 "parent", 1)

            mc.skinCluster(self.nurb_drivers[limb[0]], limb[1], mi=1, dr=10, tsb=1)

    def _createFootIkHierarchy(self):
        """Create IK foot control hierarchy. """

        ik_main_ctl = mc.circle(n="{}_{}_footIk_ctl_{}".format(self.side, self.name, self.instance_num), r=1)[0]
        ik_main_offset_grp = mc.group(ik_main_ctl, n=ik_main_ctl.replace("ctl", "grp"))
        ik_offset_ctl = mc.circle(n=ik_main_ctl.replace("_ctl", "Offset_ctl"), r=.75)[0]
        ik_offset_grp = mc.group(ik_offset_ctl, n=ik_offset_ctl.replace("ctl", "grp"))
        constrain_hock_grp = mc.group(n=ik_main_offset_grp.replace("foot", "constrainHock"), em=1, p=ik_offset_ctl)
        orient_hock_grp = mc.group(n=ik_main_offset_grp.replace("foot", "orientHock"), em=1, p=constrain_hock_grp)
        mc.parent(ik_offset_grp, ik_main_ctl)
        mc.parent(ik_main_offset_grp, self.mod_dict["IKcontrol"])

        mc.xform(ik_main_offset_grp, ws=1, t=self.guides["ball"][1])

        for i in [ik_main_ctl, ik_offset_ctl]:
            self.ik_controllers[i.split("_")[2]] = i

        self.misc["hockmaster1"] = constrain_hock_grp
        self.misc["hockslave1"] = orient_hock_grp

        mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", ik_main_offset_grp + ".v")

    def _createHipIk(self):
        """Create IK hip control hierarchy. """

        ik_main_ctl = mc.circle(n="{}_{}_hipIk_ctl_{}".format(self.side, self.name, self.instance_num), r=1)[0]
        ik_main_grp = mc.group(ik_main_ctl, n=ik_main_ctl.replace("ctl", "grp"))
        ik_main_zero = mc.group(ik_main_grp, n=ik_main_ctl.replace("ctl", "zero"))

        mc.parent(ik_main_zero, self.mod_dict["IKcontrol"])

        mc.xform(ik_main_zero, ws=1, t=self.guides["hip"][1])

        self.ik_controllers[ik_main_ctl.split("_")[2]] = ik_main_ctl

        mc.pointConstraint(ik_main_ctl, self.joint_dict["hipSPRING"],
                           n=self.joint_dict["hipSPRING"].replace("jnt", "poc"), mo=1)

        mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", ik_main_zero + ".v")

    def _createHockIkHierarchy(self):
        """Create IK hock control hierarchy. """

        ik_hock_ctl = mc.circle(n="{}_{}_hockIk_ctl_{}".format(self.side, self.name, self.instance_num), r=1)[0]
        ik_hock_grp = mc.group(ik_hock_ctl, n=ik_hock_ctl.replace("ctl", "grp"))
        ik_hock_zero = mc.group(ik_hock_grp, n=ik_hock_ctl.replace("ctl", "zero"))
        inverse_hock_grp = mc.group(n=ik_hock_ctl.replace("_ctl", "_Inverse_grp"), em=1)
        inverse_hock_zero = mc.group(inverse_hock_grp, n=ik_hock_ctl.replace("_ctl", "_Inverse_zero"))

        mc.setAttr(inverse_hock_grp + ".rotateOrder", 5)

        mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", ik_hock_zero + ".v")

        mc.parent(inverse_hock_zero, ik_hock_ctl)
        mc.parent(ik_hock_zero, self.mod_dict["IKcontrol"])

        mc.xform(ik_hock_zero, ws=1, matrix=self.guides["foot"][2])

        par_del = mc.parentConstraint(self.ik_controllers["footIk"], inverse_hock_zero)
        mc.delete(par_del)

        mult = mc.createNode("multiplyDivide", n=inverse_hock_grp.replace("grp", "mdn"))
        mc.setAttr(mult + ".input2X", -1)
        mc.setAttr(mult + ".input2Y", -1)
        mc.setAttr(mult + ".input2Z", -1)

        mc.connectAttr(self.misc["hockslave1"] + ".r", mult + ".input1")
        mc.connectAttr(mult + ".output", inverse_hock_grp + ".r")

        hock_grp = mc.listRelatives(self.ik_controllers["footIkOffset"], ad=1)[1]
        rjc.constrain_object(hock_grp, ik_hock_zero, "parent", 1)

        rollIn_grp = mc.group(n=ik_hock_ctl.replace("ctl", "rollIn_org"), em=1)
        rollOut_grp = mc.group(n=ik_hock_ctl.replace("ctl", "rollOut_org"), em=1)

        rollHeel_ctl = mc.circle(n=ik_hock_ctl.replace("ctl", "rollHeel_ctl"), r=.4)[0]
        rollHeel_cth = mc.group(rollHeel_ctl, n=ik_hock_ctl.replace("ctl", "rollHeel_cth"))
        rollHeel_org = mc.group(rollHeel_cth, n=ik_hock_ctl.replace("ctl", "rollHeel_org"))

        rollToe_ctl = mc.circle(n=ik_hock_ctl.replace("ctl", "rollToe_ctl"), r=.4)[0]
        rollToe_cth = mc.group(rollToe_ctl, n=ik_hock_ctl.replace("ctl", "rollToe_cth"))
        rollToe_org = mc.group(rollToe_cth, n=ik_hock_ctl.replace("ctl", "rollToe_org"))

        ballIk_ctl = mc.circle(n=ik_hock_ctl.replace("hockIk_ctl", "ballIk_ctl"), r=.4)[0]
        ballIk_cth = mc.group(ballIk_ctl, n=ballIk_ctl.replace("ctl", "cth"))
        ballIk_org = mc.group(ballIk_cth, n=ballIk_ctl.replace("ctl", "org"))

        footIk_grp = mc.group(n=ik_hock_ctl.replace("ctl", "footHook_grp"), em=1)
        footIk_cth = mc.group(footIk_grp, n=footIk_grp.replace("grp", "cth"))
        footIk_org = mc.group(footIk_cth, n=footIk_grp.replace("grp", "org"))
        footIkRev_org = mc.group(footIk_org, n=footIk_grp.replace("_grp", "Inverse_org"))
        footIkAttr_org = mc.group(footIkRev_org, n=footIk_grp.replace("_grp", "Attr_org"))

        mc.xform(rollIn_grp, ws=1, t=self.guides["rollIn"][1])
        mc.xform(rollOut_grp, ws=1, t=self.guides["rollOut"][1])
        mc.xform(rollHeel_org, ws=1, t=self.guides["rollHeel"][1])
        mc.xform(rollToe_org, ws=1, t=self.guides["toeEnd"][1])
        mc.xform(ballIk_org, ws=1, t=self.guides["ball"][1])
        mc.xform(footIkAttr_org, ws=1, t=self.guides["ball"][1])
        mc.xform(footIk_org, ws=1, matrix=self.guides["foot"][2])

        mc.parent(rollIn_grp, inverse_hock_grp)
        mc.parent(rollOut_grp, rollIn_grp)
        mc.parent(rollHeel_org, rollOut_grp)
        mc.parent(rollToe_org, rollHeel_ctl)
        mc.parent(ballIk_org, rollToe_ctl)
        mc.parent(footIkAttr_org, ballIk_ctl)

        mc.connectAttr(self.misc["hockslave1"] + ".r", footIkRev_org + ".r")
        mc.parentConstraint(ballIk_cth, self.joint_dict["ball" + "IK"], n=ballIk_cth.replace("cth", "prc"), mo=1)

        par = mc.parentConstraint(ballIk_cth, self.joint_dict["ball" + "FK"], self.joint_dict["ball" + "FINAL"],
                                  n=self.joint_dict["ball" + "FK"].replace("jnt", "prc").replace("parentConstraint1",
                                                                                                 ""), mo=1)[0]
        mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", par + "." + ballIk_cth + "W0")

        rev = mc.createNode("reverse", n=self.joint_dict["ball" + "FK"].replace("jnt", "rev"))
        mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", rev + ".inputX")
        mc.connectAttr(rev + ".outputX", par + "." + self.joint_dict["ball" + "FK"] + "W1")

        self.misc["footHook"] = footIk_grp
        self.misc["footHookCth"] = footIk_cth
        self.misc["footHookAttr"] = footIkAttr_org
        self.misc["rollToe"] = rollToe_cth
        self.misc["rollHeel"] = rollHeel_cth
        self.misc["rollIn"] = rollIn_grp
        self.misc["rollOut"] = rollOut_grp

    def _createFootFkHierarchy(self):
        """Create the FK chain for the leg."""

        prev_control = None
        fk_master_grp = mc.group(n="{}_{}_footFk_{}_{}".format(self.side, self.name, self.suffix, self.instance_num),
                             p=self.mod_dict["FKcontrol"], em=1)

        for i, joint in enumerate(["pelvis", "hip", "knee", "foot", "ball"]):
            ref_joint = self.joint_dict[joint + "FK"]
            pos = mc.xform(ref_joint, q=1, ws=1, matrix=1)

            fk_ctl = mc.circle(n=ref_joint.replace("jnt", "ctl"), r=1)[0]
            fk_grp = mc.group(fk_ctl, n=fk_ctl.replace("ctl", "grp"))
            fk_zero = mc.group(fk_grp, n=fk_ctl.replace("ctl", "zero"))
            mc.xform(fk_zero, ws=1, matrix=pos)
            rjc.constrain_object(fk_ctl, self.joint_dict[joint + "FK"], "point", 1)
            mc.orientConstraint(fk_ctl, self.joint_dict[joint + "FK"], n=fk_ctl + "_orc", mo=1)

            self.fk_controllers[fk_ctl.split("_")[2]] = fk_ctl

            if i == 0:
                mc.parent(fk_zero, fk_master_grp)
                prev_control = fk_ctl
            elif i == 1:
                rev = mc.createNode("reverse", n=fk_ctl.replace("ctl", "rev"))
                mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", rev + ".inputX")
                mc.connectAttr(rev + ".outputX", fk_zero + ".v")
                mc.parent(fk_zero, prev_control)
                prev_control = fk_ctl
            else:
                mc.parent(fk_zero, prev_control)
                prev_control = fk_ctl

        rjc.constrain_object(self.fk_controllers["pelvisFK"],
                             self.ik_controllers["hipIk"].replace("ctl", "zero"), "parent", 1)
        mc.parentConstraint(self.fk_controllers["pelvisFK"],
                             self.joint_dict["pelvisIK"],
                            n=self.joint_dict["pelvisIK"].replace("jnt", "prc"), mo=1)

        mult = mc.createNode("multDoubleLinear", n=self.misc["footHookCth"].replace("cth", "_rev"))
        mc.setAttr(mult + ".input2", -1)
        mc.connectAttr(self.ik_controllers["hindleg"] + ".stretchHock", mult + ".input1")
        mc.connectAttr(mult + ".output",
                       self.misc["footHookCth"] + ".tx")

    def _createIkSolvers(self):
        """Create the ik solver for the normal IK and spring IK. """

        ik_grp = mc.group(n="{}_{}_ikHandles_{}_{}".format(self.side, self.name, self.suffix, self.instance_num),
                          p=self.mod_dict["noTransform"], em=1)

        ikspring = mc.ikHandle(sj=self.joint_dict["hip" + "SPRING"],
                               ee=self.joint_dict["ball" + "SPRING"],
                               sol="ikSpringSolver",
                               n="{}_{}_hip_iks_{}".format(self.side, self.name, self.instance_num))

        ikrps = mc.ikHandle(sj=self.joint_dict["hip" + "IK"], ee=self.joint_dict["foot" + "IK"],
                            sol="ikRPsolver", n="{}_{}_hip_iss_{}".format(self.side, self.name, self.instance_num))

        for i in [ikspring, ikrps]:
            mc.parent(i[0], ik_grp)
            self.ikSolvers[i[0].split("_")[3]] = i[0]

        hock_grp = mc.listRelatives(self.ik_controllers["footIkOffset"], pa=1)[1]
        rjc.constrain_object(hock_grp, ikspring[0], "point", 1)

        auto_hock_hook = mc.group(n=self.joint_dict["footSPRING"].replace("_jnt", "hook_grp"), em=1)
        pos = mc.xform(self.joint_dict["footSPRING"], q=1, ws=1, t=1)
        mc.xform(auto_hock_hook, ws=1, t=pos)
        mc.parent(auto_hock_hook, self.joint_dict["footSPRING"])
        self.misc["hockmaster2"] = auto_hock_hook

        rjc.constrain_object(self.misc["footHook"], ikrps[0], "point", 0)

    def _createPoleVector(self):
        """Position pole vector control. """

        root_pos = self.guides["hip"][1]
        mid_pos = self.guides["knee"][1]
        end_pos = self.guides["foot"][1]

        root_joint_vec = om.MVector(root_pos)
        mid_joint_vec = om.MVector(mid_pos)
        end_joint_vec = om.MVector(end_pos)

        line = (end_joint_vec - root_joint_vec)
        point = (mid_joint_vec - root_joint_vec)

        scale_value = (line * point) / (line * line)
        proj_vec = line * scale_value + root_joint_vec

        root_to_mid_len = (mid_joint_vec - root_joint_vec).length()
        mid_to_end_len = (end_joint_vec - mid_joint_vec).length()
        total_length = mid_to_end_len + root_to_mid_len

        pole_vec_pos = (mid_joint_vec - proj_vec).normal() * total_length + mid_joint_vec

        pole_vec = '{}_{}_pole_ctl_{}'.format(self.side, self.name, self.instance_num)
        mc.circle(n=pole_vec, r=.5)
        pole_grp = mc.group(pole_vec, n=pole_vec.replace('ctl', 'grp'))
        mc.move(pole_vec_pos.x, pole_vec_pos.y, pole_vec_pos.z, pole_grp)

        rjc.constrain_object(self.ik_controllers["footIk"], pole_grp, "parent", 1)

        # parent pole grp
        mc.parent(pole_grp, self.mod_dict["IKcontrol"])
        item = pole_vec.split('_')[2]
        self.ik_controllers[item] = pole_vec

        mc.poleVectorConstraint(pole_vec, self.ikSolvers["iks"], n=self.ikSolvers["iks"].replace("_iks", "01_pvc"))
        mc.poleVectorConstraint(pole_vec, self.ikSolvers["iss"], n=self.ikSolvers["iss"].replace("_iss", "02_pvc"))

    def _constrainHockSubGroup(self):
        """Make orient constraint to change space of hock control. """

        rjc.constrain_object([self.misc["hockmaster1"], self.misc["hockmaster2"]],
                             self.misc["hockslave1"], "orient", 1)

        mc.connectAttr(self.ik_controllers["hindleg"] + ".autoHock",
                       self.misc["hockslave1"] + "." + self.misc["hockmaster2"] + "_orient_matrix_constraint")

        rev = mc.createNode("reverse", n=self.misc["hockmaster2"].replace("hook_grp", "_rev"))
        mc.connectAttr(self.ik_controllers["hindleg"] + ".autoHock", rev + ".inputX")
        mc.connectAttr(rev + ".outputX",
                       self.misc["hockslave1"] + "." + self.misc["hockmaster1"] + "_orient_matrix_constraint")

    def _stretchy(self):
        """Stretchy attribute and create connections to joints. """

        stretchy_attr = self.ik_controllers["hindleg"] + ".stretchy"
        dist_dict = {}

        stretchy_grp = mc.group(n='{}_{}_stretchy_{}_{}'.format(self.side, self.name, self.suffix, self.instance_num),
                                em=True, p=self.mod_dict["IKcontrol"])
        mc.hide(stretchy_grp)

        start_loc = mc.spaceLocator(n=self.joint_dict["hipSPRING"].replace('jnt', 'loc'))[0]
        end_loc = mc.spaceLocator(n=self.ik_controllers["footIkOffset"].replace('ctl', 'loc'))[0]

        hip_loc_static = mc.spaceLocator(n=self.joint_dict["hipFINAL"].replace('_jnt', 'Static_loc'))[0]
        knee_loc_static = mc.spaceLocator(n=self.joint_dict["kneeFINAL"].replace('_jnt', 'Static_loc'))[0]
        foot_loc_static = mc.spaceLocator(n=self.joint_dict["footFINAL"].replace('_bnd', 'Static_loc'))[0]
        ball_loc_static = mc.spaceLocator(n=self.joint_dict["ballFINAL"].replace('_bnd', 'Static_loc'))[0]

        mc.parentConstraint(self.ik_controllers["footIkOffset"], end_loc, n=end_loc.replace("loc", "prc"))
        mc.parentConstraint(self.joint_dict["hipSPRING"], start_loc, n=start_loc.replace("loc", "prc"))

        del_hip = mc.parentConstraint(self.joint_dict["hipSPRING"], hip_loc_static)
        del_knee = mc.parentConstraint(self.joint_dict["kneeSPRING"], knee_loc_static)
        del_foot = mc.parentConstraint(self.joint_dict["footSPRING"], foot_loc_static)
        del_ball = mc.parentConstraint(self.joint_dict["ballSPRING"], ball_loc_static)

        [mc.delete(x) for x in [del_hip, del_knee, del_foot, del_ball]]

        for i in [start_loc, end_loc, hip_loc_static, knee_loc_static, foot_loc_static, ball_loc_static]:
            mc.parent(i, stretchy_grp)

        dist_node = mc.createNode('distanceBetween')
        for i in ["hip", "knee", "foot"]:
            dist = mc.createNode('distanceBetween', n=self.joint_dict[i + "FINAL"].replace('_jnt', 'Static_dst'))
            dist_dict[i] = dist

        dist_pma = mc.createNode('plusMinusAverage', n=self.joint_dict["footFINAL"].replace('_bnd', 'Static_pma'))
        mult1 = mc.createNode('multiplyDivide', n=stretchy_grp.replace("_grp", "Distance_mult"))
        cond1 = mc.createNode('condition', n=mult1.replace('mult', 'cond'))
        cond2 = mc.createNode('condition',  n=cond1.replace('Distance', 'On'))

        mc.connectAttr('{}.worldMatrix'.format(start_loc), '{}.inMatrix1.'.format(dist_node))
        mc.connectAttr('{}.worldMatrix'.format(end_loc), '{}.inMatrix2.'.format(dist_node))
        mc.connectAttr('{}.worldMatrix'.format(hip_loc_static), '{}.inMatrix1.'.format(dist_dict["hip"]))
        mc.connectAttr('{}.worldMatrix'.format(knee_loc_static), '{}.inMatrix2.'.format(dist_dict["hip"]))
        mc.connectAttr('{}.worldMatrix'.format(knee_loc_static), '{}.inMatrix1.'.format(dist_dict["knee"]))
        mc.connectAttr('{}.worldMatrix'.format(foot_loc_static), '{}.inMatrix2.'.format(dist_dict["knee"]))
        mc.connectAttr('{}.worldMatrix'.format(foot_loc_static), '{}.inMatrix1.'.format(dist_dict["foot"]))
        mc.connectAttr('{}.worldMatrix'.format(ball_loc_static), '{}.inMatrix2.'.format(dist_dict["foot"]))

        mc.connectAttr('{}.distance'.format(dist_dict["hip"]), '{}.input1D[0]'.format(dist_pma))
        mc.connectAttr('{}.distance'.format(dist_dict["knee"]), '{}.input1D[1]'.format(dist_pma))
        mc.connectAttr('{}.distance'.format(dist_dict["foot"]), '{}.input1D[2]'.format(dist_pma))

        mc.setAttr('{}.operation'.format(mult1), 2)
        mc.connectAttr('{}.output1D'.format(dist_pma), '{}.input2.input2X.'.format(mult1))
        mc.connectAttr('{}.distance'.format(dist_node), '{}.input1.input1X.'.format(mult1))

        mc.setAttr('{}.operation'.format(cond1), 3)
        mc.connectAttr('{}.output.outputX'.format(mult1), '{}.firstTerm.'.format(cond1))
        mc.setAttr('{}.secondTerm.'.format(cond1), 1)
        mc.connectAttr('{}.output.outputX'.format(mult1), '{}.colorIfTrue.colorIfTrueR.'.format(cond1))

        mc.setAttr('{}.secondTerm'.format(cond2), 1)
        mc.connectAttr('{}.outColor.outColorR'.format(cond1), '{}.colorIfTrue.colorIfTrueR.'.format(cond2))
        mc.connectAttr(stretchy_attr, '{}.firstTerm.'.format(cond2))
        mc.connectAttr('{}.outColor.outColorR'.format(cond2), '{}.scale.scaleZ.'.format(self.joint_dict["hipIK"]))
        mc.connectAttr('{}.outColor.outColorR'.format(cond2), '{}.scale.scaleZ.'.format(self.joint_dict["kneeIK"]))

    def _connectFootAttrs(self):
        """Link attributes on ik foot to proper groups. """

        mc.connectAttr(self.ik_controllers["hindleg"] + ".hockRoll", self.misc["footHookAttr"] + ".rx")
        mc.connectAttr(self.ik_controllers["hindleg"] + ".toeRoll", self.misc["rollToe"] + ".rx")
        mc.connectAttr(self.ik_controllers["hindleg"] + ".heelRoll", self.misc["rollHeel"] + ".rx")
        mc.connectAttr(self.ik_controllers["hindleg"] + ".toeSwivel", self.misc["rollToe"] + ".ry")
        mc.connectAttr(self.ik_controllers["hindleg"] + ".heelSwivel", self.misc["rollHeel"] + ".ry")

        # make rollIn clamp
        roll_In_Clamp = mc.createNode('clamp', n=self.misc["rollIn"].replace('org', 'clp'))
        # set rollIn value
        mc.setAttr('{}.maxR'.format(roll_In_Clamp), 300)
        # connect from control attr into clamp
        mc.connectAttr(self.ik_controllers["hindleg"] + ".footBank", roll_In_Clamp + ".input.inputR.")
        # connect clamp to rot z of org grp
        mc.connectAttr(roll_In_Clamp + '.output.outputR', self.misc["rollIn"] + '.rotateZ')
        # make rollOut clamp
        roll_Out_Clamp = mc.createNode('clamp', n=self.misc["rollOut"].replace('org', 'clp'))
        # set rollOut value
        mc.setAttr('{}.minR'.format(roll_Out_Clamp), -300)
        # connect from control attr into clamp
        mc.connectAttr(self.ik_controllers["hindleg"] + ".footBank", roll_Out_Clamp + '.input.inputR.')
        # connect clamp to rot z of org grp
        mc.connectAttr(roll_Out_Clamp + '.output.outputR', self.misc["rollOut"] + '.rotateZ')

    def _constrainJoints(self):
        """Connect ik and fk chains to final chain."""

        for i in ["pelvis", "hip", "knee", "foot"]:

            ik_driver = None

            if not i == "foot":
                ik_driver = self.joint_dict[i + "IK"]

                orient_blend = mc.createNode("blendColors", n="{}_{}_{}JointOrient_bcl_{}".format(
                    self.side, self.name, i, self.instance_num))
                translate_blend = mc.createNode("blendColors", n="{}_{}_{}JointTranslate_bcl_{}".format(
                    self.side, self.name, i, self.instance_num))
                scale_blend = mc.createNode("blendColors", n="{}_{}_{}JointScale_bcl_{}".format(
                    self.side, self.name, i, self.instance_num))

                mc.setAttr(orient_blend + ".blender", 1)
                mc.setAttr(translate_blend + ".blender", 1)
                mc.setAttr(scale_blend + ".blender", 1)

                mc.connectAttr(ik_driver + ".t", orient_blend + ".color1")
                mc.connectAttr(self.joint_dict[i + "FK"] + ".t", orient_blend + ".color2")
                mc.connectAttr(ik_driver + ".r", translate_blend + ".color1")
                mc.connectAttr(self.joint_dict[i + "FK"] + ".r", translate_blend + ".color2")
                mc.connectAttr(ik_driver + ".s", scale_blend + ".color1")
                mc.connectAttr(self.joint_dict[i + "FK"] + ".s", scale_blend + ".color2")

                mc.connectAttr(orient_blend + ".output", self.joint_dict[i + "FINAL"] + ".t")
                mc.connectAttr(translate_blend + ".output", self.joint_dict[i + "FINAL"] + ".r")
                mc.connectAttr(scale_blend + ".output", self.joint_dict[i + "FINAL"] + ".s")

                mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", orient_blend + ".blender")
                mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", translate_blend + ".blender")
                mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", scale_blend + ".blender")

            else:
                par = mc.parentConstraint(self.misc["footHook"], self.joint_dict["footFK"],
                                          self.joint_dict["footFINAL"],
                                          n=self.joint_dict["footFK"].replace("jnt", "poc"), mo=1)
                mc.setAttr(par[0] + ".interpType", 2)

                rev = mc.createNode("reverse", n=self.joint_dict["footFK"].replace("jnt", "rev"))
                mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", par[0] + "." + self.misc["footHook"] + "W0")
                mc.connectAttr(self.ik_controllers["hindleg"] + ".ikFk", rev + ".inputX")
                mc.connectAttr(rev + ".outputX", par[0] + "." + self.joint_dict["footFK"] + "W1")

        mc.pointConstraint(self.joint_dict["hipSPRING"], self.joint_dict["hipIK"],
                           n=self.joint_dict["hipIK"].replace("jnt", "poc"), mo=1)

    def _build(self):

        self._createStructure()
        self._createJoints()
        self._createMasterControl()
        self._createNurbs()
        self._createTwistJoints()
        self._createTwistJointDrivers()
        self._constrainNurbsDrivers()
        self._createBendControls()
        self._createFootIkHierarchy()
        self._createHipIk()
        self._createHockIkHierarchy()
        self._createFootFkHierarchy()
        self._createIkSolvers()
        self._createPoleVector()
        self._constrainHockSubGroup()
        self._stretchy()
        self._connectFootAttrs()
        self._constrainJoints()