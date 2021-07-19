using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utils : MonoBehaviour
{
    public Vector3 computeLinVel(Vector3 posStart, Vector3 posEnd, float framerate)
    {
        Vector3 vel = new Vector3((posEnd[0] - posStart[0]) / framerate,
            (posEnd[1] - posStart[1]) / framerate, (posEnd[2] - posStart[2]) / framerate);

        return vel;

    }

    public Vector3 computeAngVel(Quaternion ornStart, Quaternion ornEnd, float framerate)
    {

        Quaternion difference = ornStart * Quaternion.Inverse(ornEnd);
        float angle;
        Vector3 axis;
        difference.ToAngleAxis(out angle, out axis);

        Vector3 vel = new Vector3((axis[0] * angle) / framerate, (axis[1] * angle) / framerate, (axis[1] * angle) / framerate);

        return vel;
    }

    public Vector3 computeAngVelRel(Quaternion ornStart, Quaternion ornEnd, float framerate)
    {

        Quaternion ornStartConjugate = new Quaternion(-ornStart[0], -ornStart[1], -ornStart[2], ornStart[3]);
        Quaternion q_diff = Quaternion.Inverse(ornStartConjugate) * ornEnd;

        float angle;
        Vector3 axis;
        q_diff.ToAngleAxis(out angle, out axis);

        Vector3 vel = new Vector3((axis[0] * angle) / framerate, (axis[1] * angle) / framerate, (axis[1] * angle) / framerate);

        return vel;
    }

    public static Quaternion QuaternionFromMatrix(Matrix4x4 matrix)
    {
        return Quaternion.LookRotation(matrix.GetColumn(2), matrix.GetColumn(1));
    }



}
