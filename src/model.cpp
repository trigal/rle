#include "Netica.h"
#include "NeticaEx.c"
#include <boost/math/distributions/normal.hpp>

using namespace std;
using boost::math::normal;

class LaneModel
{
    int num_lane;

    net_bn* net = NULL;
    node_bn *sensorStateT1, *sensorStateT2, *laneT1, *laneT2, *detector, *RI;
    char mesg[MESG_LEN_ns];
    int res;
    report_ns* err;

public:
    LaneModel(int n, float P1, float P2, float P3, float P4, float sigma_trans, float sigma_obs);
    ~LaneModel();
    //double * prediction();
    const float * update(float * ev_detector, float * ev_RI);

private:
    inline float normalDistributionAt(double x, double mean, double sigma);

};

LaneModel::~LaneModel()
{
    res = CloseNetica_bn (env, mesg);
    printf ("%s\n", mesg);
}

LaneModel::LaneModel(int n, float P1, float P2, float P3, float P4, float sigma_trans, float sigma_obs)
{
    num_lane = n;
    res = CloseNetica_bn (env, mesg);
    printf ("%s\n", mesg);

    env = NewNeticaEnviron_ns(NULL, NULL, NULL);
    res = InitNetica2_bn(env, mesg);
    printf ("%s\n", mesg);

    net = NewNet_bn ("LaneModel", env);


    sensorStateT1 =    NewNode_bn ("sensorStateT1", 2, net);
    sensorStateT2 =    NewNode_bn ("sensorStateT2", 2, net);
    laneT1        =    NewNode_bn ("laneT1", num_lane, net);
    laneT2        =    NewNode_bn ("laneT2", num_lane, net);
    detector      =    NewNode_bn ("detector", num_lane, net);
    RI              =    NewNode_bn ("RI", 2, net);


    SetNodeStateNames_bn (sensorStateT1,   "Working,   Not_Working");
    SetNodeStateNames_bn (sensorStateT2,   "Working,   Not_Working");
    SetNodeStateNames_bn (RI,   "Working,   Not_Working");

    for (int i = 0; i < num_lane; i++)
    {
        char buf[20];
        sprintf(buf, "Lane%d", i + 1);
        SetNodeStateName_bn (laneT1, i, buf);
        SetNodeStateName_bn (laneT2, i, buf);
        SetNodeStateName_bn (detector, i, buf);
    }


    AddLink_bn (sensorStateT1, sensorStateT2);
    AddLink_bn (laneT1,        laneT2);
    AddLink_bn (laneT1,        detector);
    AddLink_bn (sensorStateT1, detector);
    AddLink_bn (sensorStateT1, RI);

    MakeProbsUniform(sensorStateT1);
    MakeProbsUniform(laneT1);

    SetNodeProbs(sensorStateT2, "Working", P1, 1.0 - P1);
    SetNodeProbs(sensorStateT2, "Not_Working", 1.0 - P2, P2);
    for (int mean = 1; mean <= num_lane; mean++)
    {
        prob_bn* transition_pb = (prob_bn*)malloc (num_lane * sizeof (prob_bn));
        prob_bn* emission_pb = (prob_bn*)malloc (num_lane * sizeof (prob_bn));
        float sum_transition = 0.0;
        float sum_emissin = 0.0;
        for (int x = 0; x < num_lane; x++)
        {
            transition_pb[x] = normalDistributionAt(x + 1, mean, sigma_trans);
            sum_transition += transition_pb[x];

            emission_pb[x] = normalDistributionAt(x + 1, mean, sigma_obs);
            sum_emissin += emission_pb[x];
        }
        for (int x = 0; x < num_lane; x++)
        {
            transition_pb[x] = transition_pb[x] / sum_transition;

            emission_pb[x] = emission_pb[x] / sum_emissin;
        }

        char buf[20];
        sprintf(buf, "Lane%d", mean);

        state_bn transition_states[1];
        transition_states[0] = GetStateNamed_bn(buf, laneT1);
        SetNodeProbs_bn (laneT2, transition_states, transition_pb);


        state_bn emission_states[2];
        emission_states[0] = GetStateNamed_bn(buf, laneT1);
        emission_states[1] = GetStateNamed_bn("Working", sensorStateT1);
        SetNodeProbs_bn (detector, emission_states, emission_pb);

    }

    //SetNodeProbs(detector, "Lane1", "Working", 0.880536889966470, 0.119167709403898, 0.000295387219073, 0.000000013410559);
    //SetNodeProbs(detector, "Lane2", "Working", 0.106478868028914, 0.786778329216277, 0.106478868028914, 0.000263934725895 );
    //SetNodeProbs(detector, "Lane3", "Working", 0.000263934725895, 0.106478868028914, 0.786778329216277, 0.106478868028914);
    //SetNodeProbs(detector, "Lane4", "Working", 0.000000013410559, 0.000295387219073, 0.119167709403898, 0.880536889966470);
    SetNodeProbs(detector, "Lane1", "Not_Working", 0.25, 0.25, 0.25, 0.25 );
    SetNodeProbs(detector, "Lane2", "Not_Working", 0.25, 0.25, 0.25, 0.25 );
    SetNodeProbs(detector, "Lane3", "Not_Working", 0.25, 0.25, 0.25, 0.25 );
    SetNodeProbs(detector, "Lane4", "Not_Working", 0.25, 0.25, 0.25, 0.25 );

    SetNodeProbs(RI, "Working", P3, 1.0 - P3);
    SetNodeProbs(RI, "Not_Working", 1.0 - P4, P4);


    //WriteNet_bn (net,  NewFileStream_ns ("/home/cattaneod/catkin_ws/src/road_layout_estimation/LaneModel.dne", env, NULL));


    CompileNet_bn (net);
}

const float * LaneModel::update(float * ev_detector, float * ev_RI)
{
    const float * update;

    EnterNodeLikelihood_bn(detector, ev_detector);
    EnterNodeLikelihood_bn(RI, ev_RI);
    update = GetNodeBeliefs_bn (laneT1);

    const prob_bn *belief_laneT2;
    belief_laneT2 = GetNodeBeliefs_bn (laneT2);

    const prob_bn *belief_stateT2;
    belief_stateT2 = GetNodeBeliefs_bn (sensorStateT2);

    SetNodeProbs_bn(sensorStateT1, NULL, belief_stateT2);
    SetNodeProbs_bn(laneT1,  NULL,   belief_laneT2);

    RetractNodeFindings_bn(detector);
    RetractNodeFindings_bn(RI);
    CompileNet_bn (net);

    return update;
}


inline float LaneModel::normalDistributionAt(double x, double mean, double sigma)
{
    normal distribution(mean, sigma);
    return pdf (distribution, x);
}
