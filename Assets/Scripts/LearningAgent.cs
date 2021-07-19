using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;


public class LearningAgent : Agent
{

    ExpertPlayer[] expertPlayer;
    JointDriveController m_JdController;

    [Header("Expert 1 frame ahead")] public Transform hips;
    public Transform LeftUpLeg;
    public Transform LeftLeg;
    public Transform LeftFoot;
    public Transform RightUpLeg;
    public Transform RightLeg;
    public Transform RightFoot;

    // for mass calculation
    public Transform Torso;
    public Transform Spine;
    public Transform Neck;
    public Transform Head;

    public Transform LArm;
    public Transform LForeArm;
    public Transform LHand;

    public Transform RArm;
    public Transform RForeArm;
    public Transform RHand;


    [Header("Expert 10 frame ahead")] public Transform hips_10;
    public Transform LeftUpLeg_10;
    public Transform LeftLeg_10;
    public Transform LeftFoot_10;
    public Transform RightUpLeg_10;
    public Transform RightLeg_10;
    public Transform RightFoot_10;

    
    [Header("Expert 20 frame ahead")] public Transform hips_20;
    public Transform LeftUpLeg_20;
    public Transform LeftLeg_20;
    public Transform LeftFoot_20;
    public Transform RightUpLeg_20;
    public Transform RightLeg_20;
    public Transform RightFoot_20;

    [Header("Expert 30 frame ahead")] public Transform hips_30;
    public Transform LeftUpLeg_30;
    public Transform LeftLeg_30;
    public Transform LeftFoot_30;
    public Transform RightUpLeg_30;
    public Transform RightLeg_30;
    public Transform RightFoot_30;
    

    [Header("Body Parts of me")] public Transform myhips;
    public Transform myLeftUpLeg;
    public Transform myLeftLeg;
    public Transform myLeftFoot;
    public Transform myRightUpLeg;
    public Transform myRightLeg;
    public Transform myRightFoot;

    public Transform LFootFront;
    public Transform LFootBack;
    public Transform RFootFront;
    public Transform RFootBack;


    List<float> prevActions = new List<float>();
    List<Quaternion> prevRotOfMe = new List<Quaternion>();
    List<Quaternion> curRotOfMe = new List<Quaternion>();

    Vector3 expertCOM = new Vector3(0, 0, 0);
    int initialFrame = 0;




    public override void Initialize()
    {

        // checker 
        for (int i = 0; i < 24; i++)
        {
            prevActions.Add(0f);
        }

        for (int i = 0; i < 6; i++)
        {
            prevRotOfMe.Add(Quaternion.identity);
            curRotOfMe.Add(Quaternion.identity);
        }


        // expert initialize
        expertPlayer = GetComponentsInChildren<ExpertPlayer>();
        expertPlayer[0].Load_Animation(expertPlayer[0].file_path);
        expertPlayer[0].Load_Joints();
        expertPlayer[0].current_frame = 0;
        expertPlayer[0].PlayAnimation();

        expertPlayer[1].Load_Animation(expertPlayer[1].file_path);
        expertPlayer[1].Load_Joints();

        expertPlayer[2].Load_Animation(expertPlayer[2].file_path);
        expertPlayer[2].Load_Joints();

        expertPlayer[3].Load_Animation(expertPlayer[3].file_path);
        expertPlayer[3].Load_Joints();
        
        
        // me initialize
        Vector3 V_d = new Vector3(-2f, 0f, 0f);
        myhips.transform.position = hips.transform.position + V_d;
        myhips.transform.rotation = hips.transform.rotation;

        myLeftUpLeg.transform.position = LeftUpLeg.transform.position + V_d;
        myLeftUpLeg.transform.rotation = LeftUpLeg.transform.rotation;
        myLeftLeg.transform.position = LeftLeg.transform.position + V_d;
        myLeftLeg.transform.rotation = LeftLeg.transform.rotation;
        myLeftFoot.transform.position = LeftFoot.transform.position + V_d;
        myLeftFoot.transform.rotation = LeftFoot.transform.rotation;

        myRightUpLeg.transform.position = RightUpLeg.transform.position + V_d;
        myRightUpLeg.transform.rotation = RightUpLeg.transform.rotation;
        myRightLeg.transform.position = RightLeg.transform.position + V_d;
        myRightLeg.transform.rotation = RightLeg.transform.rotation;
        myRightFoot.transform.position = RightFoot.transform.position + V_d;
        myRightFoot.transform.rotation = RightFoot.transform.rotation;
        
        
        // learner initialize
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(myhips);
        m_JdController.SetupBodyPart(myLeftUpLeg);
        m_JdController.SetupBodyPart(myLeftLeg);
        m_JdController.SetupBodyPart(myLeftFoot);
        m_JdController.SetupBodyPart(myRightUpLeg);
        m_JdController.SetupBodyPart(myRightLeg);
        m_JdController.SetupBodyPart(myRightFoot);

        //Debug.Break();
        

    }

    public override void OnEpisodeBegin()
    {


        // expert load from the beginning

        /*
        expertPlayer[0].current_frame = 1;
        expertPlayer[1].current_frame = 10;
        expertPlayer[2].current_frame = 20;
        expertPlayer[3].current_frame = 30;
        */

        //initialFrame = Random.Range(0.0F, 1.0F) < 0.5F ? (int)1f : (int)480.0f;

        //initialFrame = (int)Random.Range(0f, 1000.0f);
        initialFrame = 1;
        expertPlayer[0].current_frame = initialFrame;
        expertPlayer[0].PlayAnimation();

        Vector3 V_d = new Vector3(-2f, 0f, 0f);
        myhips.transform.position = hips.transform.position + V_d;
        myhips.transform.rotation = hips.transform.rotation;

        myLeftUpLeg.transform.position = LeftUpLeg.transform.position + V_d;
        myLeftUpLeg.transform.rotation = LeftUpLeg.transform.rotation;
        myLeftLeg.transform.position = LeftLeg.transform.position + V_d;
        myLeftLeg.transform.rotation = LeftLeg.transform.rotation;
        myLeftFoot.transform.position = LeftFoot.transform.position + V_d;
        myLeftFoot.transform.rotation = LeftFoot.transform.rotation;

        myRightUpLeg.transform.position = RightUpLeg.transform.position + V_d;
        myRightUpLeg.transform.rotation = RightUpLeg.transform.rotation;
        myRightLeg.transform.position = RightLeg.transform.position + V_d;
        myRightLeg.transform.rotation = RightLeg.transform.rotation;
        myRightFoot.transform.position = RightFoot.transform.position + V_d;
        myRightFoot.transform.rotation = RightFoot.transform.rotation;


        // learner initialize
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.InitializeBodyPart(myhips);
        m_JdController.InitializeBodyPart(myLeftUpLeg);
        m_JdController.InitializeBodyPart(myLeftLeg);
        m_JdController.InitializeBodyPart(myLeftFoot);
        m_JdController.InitializeBodyPart(myRightUpLeg);
        m_JdController.InitializeBodyPart(myRightLeg);
        m_JdController.InitializeBodyPart(myRightFoot);
        

        expertPlayer[0].current_frame = initialFrame + 1;
        expertPlayer[1].current_frame = initialFrame + 10;
        expertPlayer[2].current_frame = initialFrame + 20;
        expertPlayer[3].current_frame = initialFrame + 30;

        
        /*
        // me initialize -> 이 reset이 default값으로 되어있네
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }
        */


        // sim off
        //Physics.autoSimulation = false;

        //Debug.Break();


    }


    public override void CollectObservations(VectorSensor sensor)
    {
        
        /* ==================================
                       P(t+1)
        =================================== */
        
        // 1 frame ahead -> expertPlayer[0] 
        sensor.AddObservation(posFromRootframe(1, LeftUpLeg.transform));
        sensor.AddObservation(posFromRootframe(1, LeftLeg.transform));
        sensor.AddObservation(posFromRootframe(1, LeftFoot.transform));
        sensor.AddObservation(posFromRootframe(1, RightUpLeg.transform));
        sensor.AddObservation(posFromRootframe(1, RightLeg.transform));
        sensor.AddObservation(posFromRootframe(1, RightFoot.transform));

        sensor.AddObservation(rotFromRootframe(1, LeftUpLeg.transform));
        sensor.AddObservation(rotFromRootframe(1, LeftLeg.transform));
        sensor.AddObservation(rotFromRootframe(1, LeftFoot.transform));
        sensor.AddObservation(rotFromRootframe(1, RightUpLeg.transform));
        sensor.AddObservation(rotFromRootframe(1, RightLeg.transform));
        sensor.AddObservation(rotFromRootframe(1, RightFoot.transform));

        // linear angular velocity of the current frame
        // 원래 아에 안들어가거든?


        // 10 frame ahead -> expertPlayer[1] 
        sensor.AddObservation(posFromRootframe(2, LeftUpLeg_10.transform));
        sensor.AddObservation(posFromRootframe(2, LeftLeg_10.transform));
        sensor.AddObservation(posFromRootframe(2, LeftFoot_10.transform));
        sensor.AddObservation(posFromRootframe(2, RightUpLeg_10.transform));
        sensor.AddObservation(posFromRootframe(2, RightLeg_10.transform));
        sensor.AddObservation(posFromRootframe(2, RightFoot_10.transform));

        sensor.AddObservation(rotFromRootframe(2, LeftUpLeg_10.transform));
        sensor.AddObservation(rotFromRootframe(2, LeftLeg_10.transform));
        sensor.AddObservation(rotFromRootframe(2, LeftFoot_10.transform));
        sensor.AddObservation(rotFromRootframe(2, RightUpLeg_10.transform));
        sensor.AddObservation(rotFromRootframe(2, RightLeg_10.transform));
        sensor.AddObservation(rotFromRootframe(2, RightFoot_10.transform));

        // 20 frame ahead -> expertPlayer[2] 
        sensor.AddObservation(posFromRootframe(3, LeftUpLeg_20.transform));
        sensor.AddObservation(posFromRootframe(3, LeftLeg_20.transform));
        sensor.AddObservation(posFromRootframe(3, LeftFoot_20.transform));
        sensor.AddObservation(posFromRootframe(3, RightUpLeg_20.transform));
        sensor.AddObservation(posFromRootframe(3, RightLeg_20.transform));
        sensor.AddObservation(posFromRootframe(3, RightFoot_20.transform));

        sensor.AddObservation(rotFromRootframe(3, LeftUpLeg_20.transform));
        sensor.AddObservation(rotFromRootframe(3, LeftLeg_20.transform));
        sensor.AddObservation(rotFromRootframe(3, LeftFoot_20.transform));
        sensor.AddObservation(rotFromRootframe(3, RightUpLeg_20.transform));
        sensor.AddObservation(rotFromRootframe(3, RightLeg_20.transform));
        sensor.AddObservation(rotFromRootframe(3, RightFoot_20.transform));

        // 30 frame ahead -> expertPlayer[3] 
        sensor.AddObservation(posFromRootframe(4, LeftUpLeg_30.transform));
        sensor.AddObservation(posFromRootframe(4, LeftLeg_30.transform));
        sensor.AddObservation(posFromRootframe(4, LeftFoot_30.transform));
        sensor.AddObservation(posFromRootframe(4, RightUpLeg_30.transform));
        sensor.AddObservation(posFromRootframe(4, RightLeg_30.transform));
        sensor.AddObservation(posFromRootframe(4, RightFoot_30.transform));

        sensor.AddObservation(rotFromRootframe(4, LeftUpLeg_30.transform));
        sensor.AddObservation(rotFromRootframe(4, LeftLeg_30.transform));
        sensor.AddObservation(rotFromRootframe(4, LeftFoot_30.transform));
        sensor.AddObservation(rotFromRootframe(4, RightUpLeg_30.transform));
        sensor.AddObservation(rotFromRootframe(4, RightLeg_30.transform));
        sensor.AddObservation(rotFromRootframe(4, RightFoot_30.transform));


        /* ==================================
                infomation about me
        =================================== */
        // joint pos in float -> eliminated in my code


        // joint vel in float -> joint space 
        // 8 body part
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            sensor.AddObservation(bodyPart.jointVel);
        }

        // add position
        sensor.AddObservation(posFromRootframe(0, myLeftUpLeg.transform));
        sensor.AddObservation(posFromRootframe(0, myLeftLeg.transform));
        sensor.AddObservation(posFromRootframe(0, myLeftFoot.transform));
        sensor.AddObservation(posFromRootframe(0, myRightUpLeg.transform));
        sensor.AddObservation(posFromRootframe(0, myRightLeg.transform));
        sensor.AddObservation(posFromRootframe(0, myRightFoot.transform));

        // orientation 
        sensor.AddObservation(rotFromRootframe(0, myLeftUpLeg.transform));
        sensor.AddObservation(rotFromRootframe(0, myLeftLeg.transform));
        sensor.AddObservation(rotFromRootframe(0, myLeftFoot.transform));
        sensor.AddObservation(rotFromRootframe(0, myRightUpLeg.transform));
        sensor.AddObservation(rotFromRootframe(0, myRightLeg.transform));
        sensor.AddObservation(rotFromRootframe(0, myRightFoot.transform));

        // deepmimic gave it as world space
        /*
        // lin vel 
        sensor.AddObservation(velFromRootframe(0, myLeftUpLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(0, myLeftLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(0, myLeftFoot.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(0, myRightUpLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(0, myRightLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(0, myRightFoot.GetComponent<Rigidbody>()));

        // ang vel
        sensor.AddObservation(velFromRootframe(1, myLeftUpLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(1, myLeftLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(1, myLeftFoot.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(1, myRightUpLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(1, myRightLeg.GetComponent<Rigidbody>()));
        sensor.AddObservation(velFromRootframe(1, myRightFoot.GetComponent<Rigidbody>()));
        */

        sensor.AddObservation(myLeftUpLeg.GetComponent<Rigidbody>().velocity);
        sensor.AddObservation(myLeftLeg.GetComponent<Rigidbody>().velocity);
        sensor.AddObservation(myLeftFoot.GetComponent<Rigidbody>().velocity);
        sensor.AddObservation(myRightUpLeg.GetComponent<Rigidbody>().velocity);
        sensor.AddObservation(myRightLeg.GetComponent<Rigidbody>().velocity);
        sensor.AddObservation(myRightFoot.GetComponent<Rigidbody>().velocity);

        sensor.AddObservation(myLeftUpLeg.GetComponent<Rigidbody>().angularVelocity);
        sensor.AddObservation(myLeftLeg.GetComponent<Rigidbody>().angularVelocity);
        sensor.AddObservation(myLeftFoot.GetComponent<Rigidbody>().angularVelocity);
        sensor.AddObservation(myRightUpLeg.GetComponent<Rigidbody>().angularVelocity);
        sensor.AddObservation(myRightLeg.GetComponent<Rigidbody>().angularVelocity);
        sensor.AddObservation(myRightFoot.GetComponent<Rigidbody>().angularVelocity);
        


        // up vector of the trunk
        sensor.AddObservation(myhips.transform.up);

        // the height of the skeletal root
        sensor.AddObservation(myhips.transform.position.y);

        // ground clearance at both feet
        sensor.AddObservation(LFootFront.position);
        sensor.AddObservation(LFootBack.position);
        sensor.AddObservation(RFootFront.position);
        sensor.AddObservation(RFootBack.position);

        //Debug.Break();

    }


    public override void OnActionReceived(ActionBuffers actionBuffers)
    {

        //Debug.Log("OnActionReceived");

        /* ==================================
                play expert and get vel
        =================================== */
        List<Quaternion> prevRotOfExpert = new List<Quaternion>();
        prevRotOfExpert.Add(rotFromRootframe(1, LeftUpLeg.transform));
        prevRotOfExpert.Add(rotFromRootframe(1, LeftLeg.transform));
        prevRotOfExpert.Add(rotFromRootframe(1, LeftFoot.transform));
        prevRotOfExpert.Add(rotFromRootframe(1, RightUpLeg.transform));
        prevRotOfExpert.Add(rotFromRootframe(1, RightLeg.transform));
        prevRotOfExpert.Add(rotFromRootframe(1, RightFoot.transform));

        // expert play by frame 
        // kinematic 60 fps
        expertPlayer[0].PlayAnimation();
        expertPlayer[1].PlayAnimation();
        expertPlayer[2].PlayAnimation();
        expertPlayer[3].PlayAnimation();


        List<Quaternion> curRotOfExpert = new List<Quaternion>();
        curRotOfExpert.Add(rotFromRootframe(1, LeftUpLeg.transform));
        curRotOfExpert.Add(rotFromRootframe(1, LeftLeg.transform));
        curRotOfExpert.Add(rotFromRootframe(1, LeftFoot.transform));
        curRotOfExpert.Add(rotFromRootframe(1, RightUpLeg.transform));
        curRotOfExpert.Add(rotFromRootframe(1, RightLeg.transform));
        curRotOfExpert.Add(rotFromRootframe(1, RightFoot.transform));


        List<float> expertVel = getVelocity(prevRotOfExpert, curRotOfExpert);

        /* ==================================
                me action and get vel
        =================================== */
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;
        var continuousActions = actionBuffers.ContinuousActions;

        // axis + angle from action // root로 값 받고, 먹일때 seen by parent해서 낸다
        Quaternion myLeftUpLegByRoot = actionToQuat(continuousActions[++i], continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        Quaternion myLeftUpLegByParent = myLeftUpLegByRoot;
        bpDict[myLeftUpLeg].ActionToPose(myLeftUpLegByParent);

        Quaternion myLeftLegByRoot = actionToQuat(continuousActions[++i], continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        Quaternion myLeftLegByParent = Quaternion.Inverse(Quaternion.Inverse(myhips.rotation) * myLeftUpLeg.rotation) * myLeftLegByRoot;
        bpDict[myLeftLeg].ActionToPose(myLeftLegByParent);

        Quaternion myLeftFootByRoot = actionToQuat(continuousActions[++i], continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        Quaternion myLeftFootByParent = Quaternion.Inverse(Quaternion.Inverse(myhips.rotation) * myLeftLeg.rotation) * myLeftFootByRoot;
        bpDict[myLeftFoot].ActionToPose(myLeftFootByParent);

        Quaternion myRightUpLegByRoot = actionToQuat(continuousActions[++i], continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        Quaternion myRightUpLegByParent = myRightUpLegByRoot;
        bpDict[myRightUpLeg].ActionToPose(myRightUpLegByParent);

        Quaternion myRightLegByRoot = actionToQuat(continuousActions[++i], continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        Quaternion myRightLegByParent = Quaternion.Inverse(Quaternion.Inverse(myhips.rotation) * myRightUpLeg.rotation) * myRightLegByRoot;
        bpDict[myRightLeg].ActionToPose(myRightLegByParent);

        Quaternion myRightFootByRoot = actionToQuat(continuousActions[++i], continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        Quaternion myRightFootByParent = Quaternion.Inverse(Quaternion.Inverse(myhips.rotation) * myRightLeg.rotation) * myRightFootByRoot;
        bpDict[myRightFoot].ActionToPose(myRightFootByParent);

        

        // save action for check
        List<float> actionCheck = new List<float>();

        for (int actionLength = 0; actionLength < continuousActions.Length; actionLength++){
            actionCheck.Add(continuousActions[actionLength]);
        }
        prevActions = actionCheck;

        // 6 body part
        List<float> myVel = getVelocity(prevRotOfMe, curRotOfMe);

        bpDict[myLeftUpLeg].jointVel = myVel[0];
        bpDict[myLeftLeg].jointVel = myVel[1];
        bpDict[myLeftFoot].jointVel = myVel[2];

        bpDict[myRightUpLeg].jointVel = myVel[3];
        bpDict[myRightLeg].jointVel = myVel[4];
        bpDict[myRightFoot].jointVel = myVel[5];


        /* ================================
                       reward
        ================================= */
        var jointsAtLimitPenality = jointAtLimitPenaltyFunc() * 4;

        var posReward = posRewardFunc();
        var velReward = velRewardFunc(expertVel, myVel);
        var EEPosReward = EEPosRewardFunc();
        var centerMassReward = COMRewardFunc();
        var rootPosReward = 0f;

        float posRewardScale = .7f;
        float velRewardScale = .2f;
        float EEPosRewardScale = .3f;
        float centerMassRewardScale = .3f;
        var rootPosRewardScale = 0f;

        /*
        Debug.Log("posReward " + posReward.ToString());
        Debug.Log("velReward " + velReward.ToString());
        Debug.Log("EEPosReward " + EEPosReward.ToString());
        Debug.Log("centerMassReward " + centerMassReward.ToString());
        */

        /*
        Debug.Log("posReward " + (posReward * posRewardScale).ToString());
        Debug.Log("velReward " + (velReward * velRewardScale).ToString());
        Debug.Log("EEPosReward " + (EEPosReward * EEPosRewardScale).ToString());
        Debug.Log("centerMassReward " + (centerMassReward * centerMassRewardScale).ToString());
        */


        float distanceReward =
            (posReward * posRewardScale) +
            (velReward * velRewardScale) +
            (EEPosReward * EEPosRewardScale) +
            (centerMassReward * centerMassRewardScale) + 
            (rootPosReward * rootPosRewardScale);

        //float reward = distanceReward - jointsAtLimitPenality;
        float reward = distanceReward;

        AddReward(reward);

        /* ==================================
                       terminate
        ================================== */
        // deviating from the trajectory beyond a user-specified threshold



    }

    public Quaternion actionToQuat(float angleInRad, float axisX, float axisY, float axisZ)
    {
        Quaternion generatedQuat = new Quaternion();

        Vector3 axis = new Vector3(axisX, axisY, axisZ);
        axis = Vector3.Normalize(axis);

        float angle = angleInRad * Mathf.PI * Mathf.Rad2Deg;

        generatedQuat = Quaternion.AngleAxis(angle, axis);
        
        return generatedQuat;
    }
    


    public Quaternion getJointRotationVer1(ConfigurableJoint joint)
    {

        return (Quaternion.FromToRotation(joint.axis, joint.connectedBody.transform.rotation.eulerAngles));

    }
    
    public Quaternion getJointRotationVer2()
    {
        ConfigurableJoint jointTest = myLeftUpLeg.GetComponent<ConfigurableJoint>();
        GameObject connectedBody = jointTest.connectedBody.gameObject;

        Quaternion r;

        r = Quaternion.Inverse(connectedBody.transform.rotation) * jointTest.transform.rotation;

        return r;
    }

    void FixedUpdate()
    {
        // put proper values to controller
        //var bpDict = m_JdController.bodyPartsDict;
        //m_JdController.GetCurrentJointForces();

        // update when action took place
        if(Academy.Instance.StepCount % 5 == 0)
        {
            prevRotOfMe[0] = rotFromRootframe(0, myLeftUpLeg.transform);
            prevRotOfMe[1] = rotFromRootframe(0, myLeftLeg.transform);
            prevRotOfMe[2] = rotFromRootframe(0, myLeftFoot.transform);
            prevRotOfMe[3] = rotFromRootframe(0, myRightUpLeg.transform);
            prevRotOfMe[4] = rotFromRootframe(0, myRightLeg.transform);
            prevRotOfMe[5] = rotFromRootframe(0, myRightFoot.transform);
        }

        if (Academy.Instance.StepCount % 5 == 1)
        {
            curRotOfMe[0] = rotFromRootframe(0, myLeftUpLeg.transform);
            curRotOfMe[1] = rotFromRootframe(0, myLeftLeg.transform);
            curRotOfMe[2] = rotFromRootframe(0, myLeftFoot.transform);
            curRotOfMe[3] = rotFromRootframe(0, myRightUpLeg.transform);
            curRotOfMe[4] = rotFromRootframe(0, myRightLeg.transform);
            curRotOfMe[5] = rotFromRootframe(0, myRightFoot.transform);

        }
        
        if (Physics.autoSimulation)
            return; // do nothing if the automatic simulation is enabled
        Physics.Simulate(Time.fixedDeltaTime);

    }


    public Vector3 velFromRootframe(float LinOrAng, Rigidbody target)
    {
        Vector3 changedPos = new Vector3();

        // lin
        if(LinOrAng == 0)
        {
            Transform root_transform = myhips.transform;
            changedPos = root_transform.InverseTransformVector(target.velocity - Vector3.zero);
        }
        // ang
        else
        {
            Transform root_transform = myhips.transform;
            changedPos = root_transform.InverseTransformVector(target.velocity - Vector3.zero);
        }

        return changedPos;
    }



    public Vector3 posFromRootframe(float respectToWhich, Transform target)
    {
        Vector3 changedPos = new Vector3();
        if (respectToWhich == 0)
        {
            Transform root_transform = myhips.transform;
            Quaternion jointOrnByBase = target.rotation * Quaternion.Inverse(root_transform.rotation);
            Vector3 input = (target.position - root_transform.position);
            changedPos = Quaternion.Inverse(root_transform.rotation) * input;

            /*
            Transform root_transform_parent = myhips.transform.parent;
            Vector3 changedPos_parent = Quaternion.Inverse(root_transform_parent.rotation) * input;
            Vector3 check = root_transform.InverseTransformVector(input);
            Debug.Log(changedPos - changedPos_parent);
            Debug.Log(check - changedPos);
            Debug.Log("-----------------------------------------------------");
            */

        }
        else if (respectToWhich == 1)
        {
            Transform root_transform = hips.transform;
            changedPos = Quaternion.Inverse(root_transform.rotation) * (target.position - root_transform.position);
        }
        else if (respectToWhich == 2)
        {
            Transform root_transform = hips_10.transform;
            changedPos = Quaternion.Inverse(root_transform.rotation) * (target.position - root_transform.position);
        }
        else if (respectToWhich == 3)
        {
            Transform root_transform = hips_20.transform;
            changedPos = Quaternion.Inverse(root_transform.rotation) * (target.position - root_transform.position);
        }
        else
        {
            Transform root_transform = hips_30.transform;
            changedPos = Quaternion.Inverse(root_transform.rotation) * (target.position - root_transform.position);
        }

        return changedPos;
    }

    public Quaternion rotFromRootframe(float respectToWhich, Transform target)
    {
        Quaternion changedRot = new Quaternion();
        
        if(respectToWhich == 0)
        {
            Transform root_transform = myhips.transform;
            changedRot = Quaternion.Inverse(root_transform.rotation) * target.rotation;
            
        }
        else if (respectToWhich == 1)
        {
            Transform root_transform = hips.transform;
            changedRot = Quaternion.Inverse(root_transform.rotation) * target.rotation;
        }
        else if(respectToWhich == 2)
        {
            Transform root_transform = hips_10.transform;
            changedRot = Quaternion.Inverse(root_transform.rotation) * target.rotation;
        }
        else if (respectToWhich == 3)
        {
            Transform root_transform = hips_20.transform;
            changedRot = Quaternion.Inverse(root_transform.rotation) * target.rotation;
        }
        else
        {
            Transform root_transform = hips_30.transform;
            changedRot = Quaternion.Inverse(root_transform.rotation) * target.rotation;
        }

        return changedRot;
    }


    public float posRewardFunc()
    {
        // deepmimic has weight
        /*
        mJointWeights
        0.20833, 0.10416, 0.0625, 
        0.10416, 0.0625, 0.0416, 0.0625, 0.0416, 0.00, 
        0.10416, 0.0625, 0.0416, 0.0625, 0.0416, 0.0000
        */

        float posSum = 0;

        // left
        Quaternion LeftUpLegRotDiff = rotFromRootframe(0, myLeftUpLeg) * Quaternion.Inverse(rotFromRootframe(1, LeftUpLeg));
        // LeftUpLegRotDiff.SetFromToRotation
        float LeftUpLegRotAngle;
        Vector3 LeftUpLegRotAxis;
        LeftUpLegRotDiff.ToAngleAxis(out LeftUpLegRotAngle, out LeftUpLegRotAxis);
        LeftUpLegRotAngle = Mathf.Deg2Rad * LeftUpLegRotAngle;

        Quaternion LeftLegRotDiff = rotFromRootframe(0, myLeftLeg) * Quaternion.Inverse(rotFromRootframe(1, LeftLeg));
        float LeftLegRotAngle;
        Vector3 LeftLegRotAxis;
        LeftLegRotDiff.ToAngleAxis(out LeftLegRotAngle, out LeftLegRotAxis);
        LeftLegRotAngle = Mathf.Deg2Rad * LeftLegRotAngle;

        Quaternion LeftFootRotDiff = rotFromRootframe(0, myLeftFoot) * Quaternion.Inverse(rotFromRootframe(1, LeftFoot));
        float LeftFootRotAngle;
        Vector3 LeftFootRotAxis;
        LeftFootRotDiff.ToAngleAxis(out LeftFootRotAngle, out LeftFootRotAxis);
        LeftFootRotAngle = Mathf.Deg2Rad * LeftFootRotAngle;

        // right
        Quaternion RightUpLegRotDiff = rotFromRootframe(0, myRightUpLeg) * Quaternion.Inverse(rotFromRootframe(1, RightUpLeg));
        float RightUpLegRotAngle;
        Vector3 RightUpLegRotAxis;
        RightUpLegRotDiff.ToAngleAxis(out RightUpLegRotAngle, out RightUpLegRotAxis);
        RightUpLegRotAngle = Mathf.Deg2Rad * RightUpLegRotAngle;

        Quaternion RightLegRotDiff = rotFromRootframe(0, myRightLeg) * Quaternion.Inverse(rotFromRootframe(1, RightLeg));
        float RightLegRotAngle;
        Vector3 RightLegRotAxis;
        RightLegRotDiff.ToAngleAxis(out RightLegRotAngle, out RightLegRotAxis);
        RightLegRotAngle = Mathf.Deg2Rad * RightLegRotAngle;

        Quaternion RightFootRotDiff = rotFromRootframe(0, myRightFoot) * Quaternion.Inverse(rotFromRootframe(1, myRightFoot));
        float RightFootRotAngle;
        Vector3 RightFootRotAxis;
        RightFootRotDiff.ToAngleAxis(out RightFootRotAngle, out RightFootRotAxis);
        RightFootRotAngle = Mathf.Deg2Rad * RightFootRotAngle;

        posSum = LeftUpLegRotAngle + LeftLegRotAngle + LeftFootRotAngle + RightUpLegRotAngle + RightLegRotAngle + RightFootRotAngle;

        posSum = Mathf.Exp(-1f * 2f * posSum);

        return posSum;
    }

    public float velRewardFunc(List<float> expert, List<float> me)
    {
        float velSum = 0;

        for (int i = 0; i < expert.Count; i++)
        {
            velSum = velSum + Mathf.Abs(expert[i] - me[i]);
        }

        velSum = Mathf.Exp(-1f * 2f * velSum);

        return velSum;
    }

    public List<float> getVelocity(List<Quaternion> prev, List<Quaternion> cur)
    {
        float frameRate = 1f / 30f;
        List<float> result = new List<float>();

        Quaternion LeftUpLegRotDiff = prev[0] * Quaternion.Inverse(cur[0]);
        float LeftUpLegRotAngle;
        Vector3 LeftUpLegRotAxis;
        LeftUpLegRotDiff.ToAngleAxis(out LeftUpLegRotAngle, out LeftUpLegRotAxis);
        LeftUpLegRotAngle = Mathf.Deg2Rad * LeftUpLegRotAngle;
        result.Add(LeftUpLegRotAngle / frameRate);

        Quaternion LeftLegRotDiff = prev[1] * Quaternion.Inverse(cur[1]);
        float LeftLegRotAngle;
        Vector3 LeftLegRotAxis;
        LeftLegRotDiff.ToAngleAxis(out LeftLegRotAngle, out LeftLegRotAxis);
        LeftLegRotAngle = Mathf.Deg2Rad * LeftLegRotAngle;
        result.Add(LeftLegRotAngle / frameRate);

        Quaternion LeftFootRotDiff = prev[2] * Quaternion.Inverse(cur[2]);
        float LeftFootRotAngle;
        Vector3 LeftFootRotAxis;
        LeftFootRotDiff.ToAngleAxis(out LeftFootRotAngle, out LeftFootRotAxis);
        LeftFootRotAngle = Mathf.Deg2Rad * LeftFootRotAngle;
        result.Add(LeftFootRotAngle / frameRate);

        Quaternion RightUpLegRotDiff = prev[3] * Quaternion.Inverse(cur[3]);
        float RightUpLegRotAngle;
        Vector3 RightUpLegRotAxis;
        RightUpLegRotDiff.ToAngleAxis(out RightUpLegRotAngle, out RightUpLegRotAxis);
        RightUpLegRotAngle = Mathf.Deg2Rad * RightUpLegRotAngle;
        result.Add(RightUpLegRotAngle / frameRate);

        Quaternion RightLegRotDiff = prev[4] * Quaternion.Inverse(cur[4]);
        float RightLegRotAngle;
        Vector3 RightLegRotAxis;
        RightLegRotDiff.ToAngleAxis(out RightLegRotAngle, out RightLegRotAxis);
        RightLegRotAngle = Mathf.Deg2Rad * RightLegRotAngle;
        result.Add(RightLegRotAngle / frameRate);

        Quaternion RightFootRotDiff = prev[5] * Quaternion.Inverse(cur[5]);
        float RightFootRotAngle;
        Vector3 RightFootRotAxis;
        RightFootRotDiff.ToAngleAxis(out RightFootRotAngle, out RightFootRotAxis);
        RightFootRotAngle = Mathf.Deg2Rad * RightFootRotAngle;
        result.Add(RightFootRotAngle / frameRate);

        

        return result;

    }


    // this is wrt world in original paper -> wrt to root in here
    public float EEPosRewardFunc()
    {

        Vector3 initialDiff = new Vector3(-2f, 0, 0);

        // 내 발을 expert위치에 가져다 놓는다면?

        float EEPosSum = 0;

        float rightFootPos = ((myRightFoot.transform.position - initialDiff) - RightFoot.transform.position).sqrMagnitude;
        float leftFootPos = ((myLeftFoot.transform.position - initialDiff) - LeftFoot.transform.position).sqrMagnitude;

        /*
        Vector3 add = new Vector3(0, 1, 0);
        Debug.DrawLine(myRightFoot.transform.position - initialDiff, myRightFoot.transform.position - initialDiff + add, Color.green, Time.deltaTime, false);

        if (Academy.Instance.StepCount > 1000)
        {
            Debug.Break();
        }
        */
        

        EEPosSum = rightFootPos + leftFootPos; // difference
        EEPosSum = Mathf.Exp(-1f * EEPosSum);


        return EEPosSum;
    }


    public Vector3 getCOMExpert()
    {
        
        Vector3 COM = Vector3.zero;
           
        
        COM += Torso.transform.position * 7;
        COM += Spine.transform.position * 6;
        COM += Neck.transform.position * 1;
        COM += Head.transform.position * 2;
        
        COM += LArm.transform.position * 3;
        COM += LForeArm.transform.position * 2;
        COM += LHand.transform.position * 2;

        COM += RArm.transform.position * 3;
        COM += RForeArm.transform.position * 2;
        COM += RHand.transform.position * 2;

        expertCOM = COM;

        COM += hips.transform.position * 6;

        COM += LeftUpLeg.transform.position * 5;
        COM += LeftLeg.transform.position * 3;
        COM += LeftFoot.transform.position * 1;

        COM += RightUpLeg.transform.position * 5;
        COM += RightUpLeg.transform.position * 3;
        COM += RightUpLeg.transform.position * 1;

        COM /= 54f;


        return COM;
    }

    public Vector3 getCOMme()
    {
        Vector3 COM = Vector3.zero;

        
        Vector3 moveCOM = new Vector3(-2f, 0f, 0f);
        Vector3 correctPos = new Vector3(myhips.transform.position.x - hips.transform.position.x, hips.transform.position.y/30, 
                                            myhips.transform.position.z - hips.transform.position.z);

        COM = expertCOM + (correctPos * 30);

        COM += myhips.GetComponent<Rigidbody>().worldCenterOfMass * myhips.GetComponent<Rigidbody>().mass;

        COM += myLeftUpLeg.GetComponent<Rigidbody>().worldCenterOfMass * myLeftUpLeg.GetComponent<Rigidbody>().mass;
        COM += myLeftLeg.GetComponent<Rigidbody>().worldCenterOfMass * myLeftLeg.GetComponent<Rigidbody>().mass;
        COM += myLeftFoot.GetComponent<Rigidbody>().worldCenterOfMass * myLeftFoot.GetComponent<Rigidbody>().mass;

        COM += myRightUpLeg.GetComponent<Rigidbody>().worldCenterOfMass * myRightUpLeg.GetComponent<Rigidbody>().mass;
        COM += myRightLeg.GetComponent<Rigidbody>().worldCenterOfMass * myRightLeg.GetComponent<Rigidbody>().mass;
        COM += myRightFoot.GetComponent<Rigidbody>().worldCenterOfMass * myRightFoot.GetComponent<Rigidbody>().mass;

        COM /= 54f;

        /*
        Vector3 add = new Vector3(0, 1, 0);
        Debug.DrawLine(COM / 30, COM / 30 + add, Color.green, Time.deltaTime, false);

        if (Academy.Instance.StepCount > 10)
        {
            Debug.Break();
        }
        */

        

        return COM;
    }

    public float COMRewardFunc()
    {
        float COMReward = 0;

        Vector3 initialDiff = new Vector3(-2f, 0, 0);
        Vector3 expertCOM = getCOMExpert();
        Vector3 meCOM = getCOMme();
        meCOM = meCOM - initialDiff;

        COMReward = Vector3.Distance(expertCOM, meCOM);
        COMReward = Mathf.Exp(-5f * COMReward);


        return COMReward;
    }


    public float jointAtLimitPenaltyFunc()
    {

        float countOutofLimit = 0;

        // 하나씩 돌면서 normalized 된 x, y, z값이 1보다 크면 penalty 넣어줘야함
        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            if(Mathf.Abs(bodyPart.currentXNormalizedRot) >= 1f)
            {
                countOutofLimit++;
            }
            if (Mathf.Abs(bodyPart.currentYNormalizedRot) >= 1f)
            {
                countOutofLimit++;
            }
            if (Mathf.Abs(bodyPart.currentZNormalizedRot) >= 1f)
            {
                countOutofLimit++;
            }

        }

        float penalty = countOutofLimit * 0.2f;

        return penalty;

    }





}
