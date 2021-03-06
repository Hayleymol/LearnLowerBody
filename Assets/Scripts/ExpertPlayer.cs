using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;


public class ExpertPlayer : MonoBehaviour
{

    public int target_fps = 60;

    public enum Representation {
        Local,
        World,
        Reflocal,
        Refworld,
        Body_vel,
        Ref_vel
    };

    public string file_path;
    public Representation representation;
    float[,] Animation_World;
    float[,] Animation;
    public int Length;

    public GameObject Root;
    public List<GameObject> Joints;

    public int current_frame = 0;

    public bool vel_updated;

    

    //private void FixedUpdate()
    public void PlayAnimation()
    {
        
        if (true)
        { // local world reflocal
            if (representation != Representation.Body_vel && representation != Representation.Ref_vel)
            {
                
                //Debug.Log(Joints.Count);// 지금 0 나옴
                for (int j = 0; j < Joints.Count; j++)
                {
                    Vector3 up = new Vector3(Animation[current_frame, 9 * j]
                              , Animation[current_frame, 9 * j + 3]
                              , Animation[current_frame, 9 * j + 6]);
                    Vector3 forward = new Vector3(Animation[current_frame, 9 * j + 1]
                            , Animation[current_frame, 9 * j + 4]
                            , Animation[current_frame, 9 * j + 7]);
                    Vector3 pos = new Vector3(Animation[current_frame, 9 * j + 2]
                            , Animation[current_frame, 9 * j + 5]
                            , Animation[current_frame, 9 * j + 8]);

                    if (representation == Representation.World)
                    {
                        Joints[j].transform.position = pos;
                        Joints[j].transform.rotation = Quaternion.LookRotation(forward, up);
                    }
                    else if (representation == Representation.Refworld)
                    {
                        if (j == 0)
                        {
                            Joints[j].transform.position = pos;
                            Joints[j].transform.rotation = Quaternion.LookRotation(forward, up);
                        }
                        else
                        {
                            Joints[j].transform.position = Root.transform.position + Root.transform.TransformVector(pos);
                            Joints[j].transform.rotation = Root.transform.rotation * Quaternion.LookRotation(forward, up);
                        }
                    }
                    else
                    {
                        Joints[j].transform.localPosition = pos;
                        Joints[j].transform.localRotation = Quaternion.LookRotation(forward, up);

                        //Joints[j].transform.localRotation = Quaternion.AngleAxis(forward, up);
                        
                    }
                }
            }
            else
            { // velocities
                if (current_frame == 0)
                {
                    for (int j = 0; j < Joints.Count; j++)
                    {
                        Vector3 up = new Vector3(Animation_World[current_frame, 9 * j]
                                  , Animation_World[current_frame, 9 * j + 3]
                                  , Animation_World[current_frame, 9 * j + 6]);
                        Vector3 forward = new Vector3(Animation_World[current_frame, 9 * j + 1]
                                , Animation_World[current_frame, 9 * j + 4]
                                , Animation_World[current_frame, 9 * j + 7]);
                        Vector3 pos = new Vector3(Animation_World[current_frame, 9 * j + 2]
                                , Animation_World[current_frame, 9 * j + 5]
                                , Animation_World[current_frame, 9 * j + 8]);

                        Joints[j].transform.position = pos;
                        Joints[j].transform.rotation = Quaternion.LookRotation(forward, up);
                    }
                }
                else
                {
                    if (!vel_updated)
                    {
                        for (int j = 0; j < Joints.Count; j++)
                        {
                            Vector3 up = new Vector3(Animation[current_frame, 9 * j]
                                      , Animation[current_frame, 9 * j + 3]
                                      , Animation[current_frame, 9 * j + 6]);
                            Vector3 forward = new Vector3(Animation_World[current_frame, 9 * j + 1]
                                    , Animation[current_frame, 9 * j + 4]
                                    , Animation[current_frame, 9 * j + 7]);
                            Vector3 pos = new Vector3(Animation_World[current_frame, 9 * j + 2]
                                    , Animation[current_frame, 9 * j + 5]
                                    , Animation[current_frame, 9 * j + 8]);

                            if (representation == Representation.Body_vel)
                            {
                                Joints[j].transform.position = Joints[j].transform.position + pos;
                                Joints[j].transform.rotation *= Quaternion.LookRotation(forward, up);
                            }
                            else
                            {
                                continue;
                            }
                        }
                        vel_updated = true;
                    }
                }
            }

            if (current_frame == Animation.GetLength(0) - 1)
                current_frame = 0;
            else
            {
                current_frame++;
                vel_updated = false;
            }
        }
    }


   
    public void Load_Joints()
    {
        if (Animation == null)
            Load_Animation(file_path);

        Joints.Clear();

        foreach (Transform child in Root.GetComponentsInChildren<Transform>())
        {
            if (child.name != "Sphere" && child.name != "Cylinder" && child.name != "Cube")
                Joints.Add(child.gameObject);
        }


        //Debug.Log(Animation.GetLength(1) / 9); // 이게 14개밖에 안되는데?
        if (Animation.GetLength(1) / 9 != Joints.Count)
            
            Debug.LogError("Joint count not match!");
    }

    

    public void Load_Animation(string path)
    {
        string original_path = path;

        switch (representation)
        {
            case Representation.Local:
                path += "_local";
                break;
            case Representation.World:
                path += "_world";
                break;
            case Representation.Reflocal:
                path += "_reflocal";
                break;
            case Representation.Refworld:
                path += "_refworld";
                break;
            case Representation.Body_vel:
                path += "_body_vel";
                break;
            case Representation.Ref_vel:
                path += "_ref_vel";
                break;
        }

        var Data = System.IO.File.ReadAllText(Application.dataPath + path + ".csv");

        var lines = Data.Split("\n"[0]);
        int row_count = lines.Length;
        var lineData = (lines[0].Trim()).Split(","[0]);
        int column_count = lineData.Length;
        var x = 0f;
        float.TryParse(lineData[0], out x);

        Animation = new float[row_count, column_count];
        Length = row_count;

        for (int i = 0; i < row_count - 1; i++)
        {
            var Line = (lines[i].Trim()).Split(","[0]);
            for (int j = 0; j < column_count; j++)
            {
                var v = 0f;
                float.TryParse(Line[j], out v);
                Animation[i, j] = v;
            }
        }
    }

}
