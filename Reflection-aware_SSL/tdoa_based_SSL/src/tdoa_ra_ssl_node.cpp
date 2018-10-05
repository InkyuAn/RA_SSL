#include <ros/ros.h>
#include "global_value.h"
#include <tdoa_based_ssl_message/PotentialSources.h>
#include <tdoa_based_ssl_message/SceneState.h>
#include <tdoa_based_ssl_message/sourceGroundTruth.h>

#include <rt_audio_ros/AudioStream.h>
#include <tf/transform_listener.h>

//Standard C includes
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <fstream>
#include <sys/stat.h>

#include "manyears_config.hpp"

using namespace std;

#include "dsplib/overallContext.h"
#include "dsplib/Preprocessing/preprocessor.h"
#include "dsplib/Localisation/beamformer.h"
#include "dsplib/Localisation/potentialSources.h"

/// For getting a current directory
#include <unistd.h>
#define GetCurrentDir getcwd

//#define ONLY_TDOA
#define ONLY_SEPARATION
//#define ORIGINAL_VERSION

namespace sound_localization_node{
    enum {Clapping_290_0_0, Voice_290_0_0,
         Clapping_290_55_0, Voice_290_55_0,
         Clapping_290_110_0, Voice_290_110_0,
         Clapping_inv_200_0_0, Voice_inv_200_0_0,
         IR_270_0_30,
         Moving_clapping_270_0_0,
         Moving_voice_270_0_0,
         Moving_voice_Obs_155_140_0,
         Moving_voice_Obs_155_110_0};


    class sound_localization
    {
    public:
        sound_localization(ros::NodeHandle n): local_nh_("~"),
            pub_(n.advertise<tdoa_based_ssl_message::PotentialSources>("dir_info",1000)),
            //pubSceneState(n.advertise<tdoa_based_ssl_message::SceneState>("Scene_State", 100, true)),
            pubSourcePos(n.advertise<tdoa_based_ssl_message::sourceGroundTruth>("source_ground_truth", 100, true)),
            output_file_origin(NULL),
            output_file_robot_pos(NULL),
            filePtr_(NULL), filePtr_sourcePos(NULL)
        {
            for(int i=0 ; i<8 ; i++)
                output_file_[i] = NULL;

            sequence = 0;

            //init variables
            ros_init_time_ = ros::Time::now();
            ros_audio_get_time_ = ros::Time::now();
            frame_number_ = 0;

            //set data to zero
            memset(audio_raw_data_, 0, manyears_global::raw_buffer_size_s * sizeof(short));
            source_position[0] = 0.0;
            source_position[1] = 0.0;
            source_position[2] = 0.0;

            //Init ROS connection and parameters
            local_nh_.param("system_delay", system_delay_, 1);
            local_nh_.param("scene_state", scene_state_, (int)Clapping_inv_200_0_0);
            local_nh_.param("save_results", save_result_, false);            


            local_nh_.param("frame_id", frame_id_, std::string(manyears_global::microphones_frame_s));
            local_nh_.param("instant_time", instant_time_, false);
            local_nh_.param("iterative_path", iterative_path_, std::string(""));
            local_nh_.param("iterative_delay", iterative_delay_, 1);
            local_nh_.param("iterative_enable", iterative_enable_, false);


            local_nh_.param("gain_sep", gain_sep_, 50.0);
            local_nh_.param("gain_pf",  gain_pf_,  50.0);

            if (instant_time_)
                ROS_INFO("Using instantaneous system time for tracked sources.");
            else
                ROS_INFO("Using estimated time from audio stream for tracked sources.");

            //Separation enable
            local_nh_.param("enable_separation", enable_separation_, false);


            float duration = (float)manyears_global::samples_per_frame_s / (float)manyears_global::sample_rate_s;
            duration = duration* ((float)system_delay_);

            std::string raw_file = this->getFilePath(scene_state_);

            ROS_INFO_STREAM("File path: "<< raw_file);
            ROS_INFO_STREAM("Duration is "<< duration);
            ROS_INFO_STREAM("Scene state "<< scene_state_ << ", Does save results? " << save_result_);
            //Open file in binary mode
            filePtr_ = fopen(raw_file.c_str(), "rb");

            /// For the moving source
            if(scene_state_ >= Moving_clapping_270_0_0){
                std::string raw_file_source_pos = this->getSourcePosFilePath(scene_state_);
                filePtr_sourcePos = fopen(raw_file_source_pos.c_str(), "rb");
            }else{
                filePtr_sourcePos = NULL;
            }


            timer_ = n.createTimer(ros::Duration(duration), &sound_localization::saved_audio_stream_cb, this);
            if (filePtr_ == NULL)
            {
                ROS_ERROR("Invalid file %s \n",raw_file.c_str());
            }
            is_audio_saved_ = false;


            libraryContext_ = createEmptyOverallContext();
            //libraryContext_.myParameters = new struct ParametersStruct;

            //Default parameters
            ParametersLoadDefault(libraryContext_.myParameters);


            if (iterative_enable_) {
                // Iterative interval delay
                libraryContext_.myParameters->P_OUT_INTERVALDURATION = iterative_delay_ * GLOBAL_FS;
            }

            //This is a copy of what is in myParameters for now
            setup_microphone_positions_and_gains(libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the preprocessor
            preprocessorInit(libraryContext_.myPreprocessor, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the beamformer
            beamformerInit(libraryContext_.myBeamformer, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the mixture
            mixtureInit(libraryContext_.myMixture, libraryContext_.myParameters);

            // Initialize the gss
            gssInit(libraryContext_.myGSS, libraryContext_.myParameters, libraryContext_.myMicrophones);

            // Initialize the postfilter
            postfilterInit(libraryContext_.myPostfilter, libraryContext_.myParameters);

            // Initialize the postprocessor
            postprocessorInit(libraryContext_.myPostprocessorSeparated, libraryContext_.myParameters);
            postprocessorInit(libraryContext_.myPostprocessorPostfiltered, libraryContext_.myParameters);

            // Initialize the potential sources          
            potentialSourcesInit(libraryContext_.myPotentialSources, libraryContext_.myParameters);

            // Initialize the sources
            trackedSourcesInit(libraryContext_.myTrackedSources, libraryContext_.myParameters);


            // Initialize the separated sources
            separatedSourcesInit(libraryContext_.mySeparatedSources, libraryContext_.myParameters);

            // Initialize the postfiltered sources
            postfilteredSourcesInit(libraryContext_.myPostfilteredSources, libraryContext_.myParameters);


        }
        ~sound_localization()
        {

            ROS_INFO("~many_ears");

            //terminate
            preprocessorTerminate(libraryContext_.myPreprocessor);
            beamformerTerminate(libraryContext_.myBeamformer);            
            mixtureTerminate(libraryContext_.myMixture);
            gssTerminate(libraryContext_.myGSS);
            postfilterTerminate(libraryContext_.myPostfilter);
            postprocessorTerminate(libraryContext_.myPostprocessorSeparated);
            postprocessorTerminate(libraryContext_.myPostprocessorPostfiltered);

            potentialSourcesTerminate(libraryContext_.myPotentialSources);            
            trackedSourcesTerminate(libraryContext_.myTrackedSources);
            separatedSourcesTerminate(libraryContext_.mySeparatedSources);
            postfilteredSourcesTerminate(libraryContext_.myPostfilteredSources);

            //Close file (will cleanup memory)
            if(filePtr_ != NULL)
                fclose(filePtr_);
            if(filePtr_sourcePos != NULL)
                fclose(filePtr_sourcePos);

            if(is_audio_saved_ == true){
                for(int i=0 ; i<manyears_global::nb_microphones_s ; i++)
                    fclose(output_file_[i]);
            }
            if(output_file_origin != NULL)
                fclose(output_file_origin);

            if(output_file_robot_pos != NULL)
                fclose(output_file_robot_pos);

        }

        void terminate()
        {
            tdoa_based_ssl_message::PotentialSources potential_source_msg;
            //Fill the message header
            //Calculate the estimated time when the trame was catch by the microphones

            potential_source_msg.header.stamp = getTimeStamp();            
            potential_source_msg.header.frame_id = frame_id_;


            pub_.publish(potential_source_msg);

        }

        ros::Time getTimeStamp()
        {
            if (instant_time_)
                return ros::Time::now();
            else
                return ros_init_time_ + ros::Duration(((float)manyears_global::samples_per_frame_s/manyears_global::sample_rate_s)*frame_number_);
        }


        void processing_buffer()
        {

            //#1 - Let's create the float data for processing
            for (int channel = 0; channel < manyears_global::nb_microphones_s; channel++)
            {
                for (int frame_index = 0; frame_index < manyears_global::samples_per_frame_s; frame_index++)
                {
                    audio_float_data_[channel][frame_index] = (float) audio_raw_data_[channel + (manyears_global::nb_microphones_s * frame_index)];// / 32768.0;

                    audio_float_data_[channel][frame_index] /= 32768.0;

                }

                // Copy frames to the beamformer frames, will do 50% overlap internally
                preprocessorPushFrames(libraryContext_.myPreprocessor, manyears_global::samples_per_frame_s, channel);
                        // manyears_global::samples_per_frame_s: 512
                preprocessorAddFrame(libraryContext_.myPreprocessor, &audio_float_data_[channel][0], channel, manyears_global::samples_per_frame_s);
            }

            //#2 Preprocess
            preprocessorProcessFrame(libraryContext_.myPreprocessor);

            //#3 Find potential sources from the beamformer
            beamformerFindMaxima(libraryContext_.myBeamformer, libraryContext_.myPreprocessor, libraryContext_.myPotentialSources);

            // Update Track sources (=Potential source)
            idListReset(&libraryContext_.myTrackedSources->sourcesID);

            for (int source_index=0 ; source_index < libraryContext_.myBeamformer->BF_MAXSOURCES ; source_index++)
            {
                if(potentialSourcesGetProbability(libraryContext_.myPotentialSources, source_index) > 0.7f){

                    // Adding Tracked sources
                    unsigned int indexID = idListAdd(&libraryContext_.myTrackedSources->sourcesID, source_index+1);

                    libraryContext_.myTrackedSources->sourcesPosition[indexID][0] = potentialSourcesGetX(libraryContext_.myPotentialSources, source_index);
                    libraryContext_.myTrackedSources->sourcesPosition[indexID][1] = potentialSourcesGetY(libraryContext_.myPotentialSources, source_index);
                    libraryContext_.myTrackedSources->sourcesPosition[indexID][2] = potentialSourcesGetZ(libraryContext_.myPotentialSources, source_index);

                }

            }

            if (enable_separation_)
            {

                //#5 Separate sources
                gssProcess(libraryContext_.myGSS, libraryContext_.myPreprocessor, libraryContext_.myTrackedSources, libraryContext_.mySeparatedSources);
                postfilterProcess(libraryContext_.myPostfilter, libraryContext_.mySeparatedSources, libraryContext_.myPreprocessor, libraryContext_.myPostfilteredSources);

                //#6 Postprocess
                postprocessorProcessFrameSeparated(libraryContext_.myPostprocessorSeparated, libraryContext_.myTrackedSources, libraryContext_.mySeparatedSources);
                postprocessorProcessFramePostfiltered(libraryContext_.myPostprocessorPostfiltered, libraryContext_.myTrackedSources, libraryContext_.myPostfilteredSources);

            }

            tdoa_based_ssl_message::PotentialSources tracked_source_msg;

            //Fill the message header
            //Calculate the estimated time when the trame was catch by the microphones
            tracked_source_msg.header.stamp = getTimeStamp();

            //tracked_source_msg.header.stamp = ros_init_time_ + ros::Duration((GLOBAL_FRAMESIZE*GLOBAL_OVERLAP/GLOBAL_FS)*frame_number_);
            tracked_source_msg.header.frame_id = frame_id_;

            // Add pick frequency band into message
            // 2018 04 17 IK
            tracked_source_msg.frequency = potentialSourceGetFrequency(libraryContext_.myPotentialSources);



            //Search for tracked source
            for (int source_index = 0; source_index < libraryContext_.myParameters->P_GEN_DYNSOURCES; source_index++)
            {
                if(trackedSourcesGetID(libraryContext_.myTrackedSources,source_index) != -1)
                {
                    //std::cout << "  found trackedSourceID" << std::endl;
                    //Fill an instance of source info msg
                    //tdoa_based_ssl_message
                    tdoa_based_ssl_message::SourceInfo tracked_source;
                    tracked_source.source_id = trackedSourcesGetID(libraryContext_.myTrackedSources,source_index);
                    tracked_source.source_pos.x = trackedSourcesGetX(libraryContext_.myTrackedSources, source_index);
                    tracked_source.source_pos.y = trackedSourcesGetY(libraryContext_.myTrackedSources, source_index);
                    tracked_source.source_pos.z = trackedSourcesGetZ(libraryContext_.myTrackedSources, source_index);
                    tracked_source.longitude = atan2f(tracked_source.source_pos.y, tracked_source.source_pos.x) * 180/M_PI;
                    tracked_source.latitude = asinf(tracked_source.source_pos.z/
                                                    (tracked_source.source_pos.z*tracked_source.source_pos.z +
                                                     tracked_source.source_pos.y*tracked_source.source_pos.y +
                                                     tracked_source.source_pos.x*tracked_source.source_pos.x )) * 180/M_PI;



                    if (enable_separation_)
                    {

                        int size = (int)((float)GLOBAL_FRAMESIZE * GLOBAL_OVERLAP);


                        //Fill separation data for the source
                        tracked_source.separation_data.resize(size);
                        tracked_source.postfiltered_data.resize(size);


                        postprocessorExtractHop(libraryContext_.myPostprocessorSeparated,
                                                tracked_source.source_id, &tracked_source.separation_data[0]);

                        postprocessorExtractHop(libraryContext_.myPostprocessorPostfiltered,
                                                tracked_source.source_id, &tracked_source.postfiltered_data[0]);

                        //Apply GAIN
                        for (int j = 0; j < size; j++)
                        {
                            tracked_source.separation_data[j] *= gain_sep_;
                            tracked_source.postfiltered_data[j] *= gain_pf_;
                        }

                    }


                    tracked_source.source_probability =
                        potentialSourcesGetProbability(libraryContext_.myPotentialSources, source_index);
                    //Add it to the tracked sources

                    tracked_source.source_energy = potentialSourcesGetEnergy(libraryContext_.myPotentialSources, source_index);

                    //tracked_source_msg.tracked_sources.push_back(tracked_source);
                    tracked_source_msg.potential_sources.push_back(tracked_source);
                }

            }

            tracked_source_msg.count = sequence++;
            pub_.publish(tracked_source_msg);

            frame_number_++;
        }


    private:
        void read_file_for_load_param(std::string& file_param_name)
        {
            ROS_INFO("Loading param file %s...",file_param_name.c_str());
            if (!sound_localization_node::parseConfigFile(libraryContext_,
                                               file_param_name)) {
                ROS_ERROR("Could not parse config file.");
            }
        }
        void setup_microphone_positions_and_gains(struct ParametersStruct *parametersStruct, struct objMicrophones* myMicrophones)
        {

            // Set the number of microphones
            microphonesInit(myMicrophones, 8);

            // Add microphone 1...
            microphonesAdd(myMicrophones,
                           0,
                           parametersStruct->P_GEO_MICS_MIC1_X,
                           parametersStruct->P_GEO_MICS_MIC1_Y,
                           parametersStruct->P_GEO_MICS_MIC1_Z,
                           parametersStruct->P_GEO_MICS_MIC1_GAIN
                           );

            // Add microphone 2...
            microphonesAdd(myMicrophones,
                           1,
                           parametersStruct->P_GEO_MICS_MIC2_X,
                           parametersStruct->P_GEO_MICS_MIC2_Y,
                           parametersStruct->P_GEO_MICS_MIC2_Z,
                           parametersStruct->P_GEO_MICS_MIC2_GAIN
                           );

            // Add microphone 3...
            microphonesAdd(myMicrophones,
                           2,
                           parametersStruct->P_GEO_MICS_MIC3_X,
                           parametersStruct->P_GEO_MICS_MIC3_Y,
                           parametersStruct->P_GEO_MICS_MIC3_Z,
                           parametersStruct->P_GEO_MICS_MIC3_GAIN
                           );

            // Add microphone 4...
            microphonesAdd(myMicrophones,
                           3,
                           parametersStruct->P_GEO_MICS_MIC4_X,
                           parametersStruct->P_GEO_MICS_MIC4_Y,
                           parametersStruct->P_GEO_MICS_MIC4_Z,
                           parametersStruct->P_GEO_MICS_MIC4_GAIN
                           );

            // Add microphone 5...
            microphonesAdd(myMicrophones,
                           4,
                           parametersStruct->P_GEO_MICS_MIC5_X,
                           parametersStruct->P_GEO_MICS_MIC5_Y,
                           parametersStruct->P_GEO_MICS_MIC5_Z,
                           parametersStruct->P_GEO_MICS_MIC5_GAIN
                           );

            // Add microphone 6...
            microphonesAdd(myMicrophones,
                           5,
                           parametersStruct->P_GEO_MICS_MIC6_X,
                           parametersStruct->P_GEO_MICS_MIC6_Y,
                           parametersStruct->P_GEO_MICS_MIC6_Z,
                           parametersStruct->P_GEO_MICS_MIC6_GAIN
                           );

            // Add microphone 7...
            microphonesAdd(myMicrophones,
                           6,
                           parametersStruct->P_GEO_MICS_MIC7_X,
                           parametersStruct->P_GEO_MICS_MIC7_Y,
                           parametersStruct->P_GEO_MICS_MIC7_Z,
                           parametersStruct->P_GEO_MICS_MIC7_GAIN
                           );

            // Add microphone 8...
            microphonesAdd(myMicrophones,
                           7,
                           parametersStruct->P_GEO_MICS_MIC8_X,
                           parametersStruct->P_GEO_MICS_MIC8_Y,
                           parametersStruct->P_GEO_MICS_MIC8_Z,
                           parametersStruct->P_GEO_MICS_MIC8_GAIN
                           );


        }

        bool fill_buffer_data_with_raw_file()
        {
            //This does not work well because it does not take account of the channel order
            //bool success = (bool) fread(audio_raw_data_,sizeof(short),manyears_global::raw_buffer_size_s,filePtr_);

            signed short inputShort;
            unsigned int indexBuffer;
            bool success;

            indexBuffer = 0;
            success = true;

            /// For moving source
            if(scene_state_ >= Moving_clapping_270_0_0){
                if (feof(this->filePtr_sourcePos)  == 0){
                    double tmpPos[3] = {0.0, };
                    int err = fread(&tmpPos, sizeof(double), 3, this->filePtr_sourcePos);
                    if( err == 0)
                        success = false;

                    source_position[0] = tmpPos[0];
                    source_position[1] = tmpPos[1];
                    source_position[2] = tmpPos[2];
                }
                else{
                    success = false;
                }

            }
            else{
                double tmpPos[3] = {0.0f};
                this->getGroundTruthPosition(scene_state_, tmpPos[0], tmpPos[1], tmpPos[2]);

                source_position[0] = tmpPos[0];
                source_position[1] = tmpPos[1];
                source_position[2] = tmpPos[2];
            }

            // Load the samples for a complete hop size
            for (int indexSample = 0; indexSample < manyears_global::samples_per_frame_s; indexSample++)
            {
                for (int indexChannel = 0; indexChannel < manyears_global::nb_microphones_s; indexChannel++)
                {
                    if (feof(this->filePtr_)  == 0)
                    {
                        int e = fread(&inputShort, sizeof(short), 1, this->filePtr_);
                        if( e == 0)
                            continue;
                    }
                    else
                    {
                        inputShort = 0;
                        success = false;
                    }
                    this->audio_raw_data_[indexBuffer] = inputShort;
                    indexBuffer++;
                }
            }

            return success;
        }

        void saved_audio_stream_cb(const ros::TimerEvent& event){

            bool read_audio_stream = fill_buffer_data_with_raw_file();

            if(read_audio_stream){
                ros_audio_get_time_ = event.current_real;
                /// publish source ground truth position
                ///
                tdoa_based_ssl_message::sourceGroundTruth sourceGroundTruthMsg;
                sourceGroundTruthMsg.source_pos.x = source_position[0];
                sourceGroundTruthMsg.source_pos.y = source_position[1];
                sourceGroundTruthMsg.source_pos.z = source_position[2];
                pubSourcePos.publish(sourceGroundTruthMsg);

                processing_buffer();
            }
            else{
                ROS_ERROR("Could not read audio stream.");

                if(save_result_){
                    if(scene_state_ >= Moving_voice_Obs_155_110_0){
                        return;
                    }
                    scene_state_++;

                    if(filePtr_ != NULL)
                        fclose(filePtr_);

                    //tdoa_based_ssl_message::SceneState tmpSceneState;
                    //tmpSceneState.scene_state = scene_state_;
                    //pubSceneState.publish(tmpSceneState);

                    std::string raw_file = this->getFilePath(scene_state_);
                    ROS_INFO_STREAM("Use "<< raw_file << " ''raw file''\n");
                    ROS_INFO_STREAM("Scene state "<< scene_state_ << "\n");
                    //Open file in binary mode
                    filePtr_ = fopen(raw_file.c_str(), "rb");

                    if (filePtr_ == NULL)
                    {
                        ROS_ERROR("Invalid file %s \n",raw_file.c_str());
                    }
                }

            }

        }

        std::string currentDateTime() {
            time_t     now = time(0);
            struct tm  tstruct;
            char       buf[80];
            tstruct = *localtime(&now);

            // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
            // for more information about date/time format
            strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

            for(int i=0 ; i<sizeof(buf) ; i++)
            {
                if(buf[i] == '.')
                    buf[i] = '-';
                if(buf[i] == ':')
                    buf[i] = '_';
            }

            return buf;
        }


        std::string getFilePath(int idx){

            std::string projectPath("/home/inkyu/catkin_ssl_ws/src/tdoa_based_SSL/localization");
            switch(idx){
            case Clapping_290_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/0_Clapping_290_0_0/origin_audio_stream");
                break;
            case Voice_290_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/1_Voice_290_0_0/origin_audio_stream");
                break;
            case Clapping_290_55_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/2_Clapping_290_55_0/origin_audio_stream");
                break;
            case Voice_290_55_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/3_Voice_290_55_0/origin_audio_stream");
                break;
            case Clapping_290_110_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/4_Clapping_290_110_0/origin_audio_stream");
                break;
            case Voice_290_110_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/5_Voice_290_110_0/origin_audio_stream");
                break;
            case Clapping_inv_200_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/6_Clapping_inv_200_0_0/origin_audio_stream");
                break;
            case Voice_inv_200_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_clapping_voice/7_Voice_inv_200_0_0/origin_audio_stream");
                break;
            case IR_270_0_30:
                return projectPath + std::string("/data/raw_data/raw_data_IR_5_times/3_IR_270_0_30/origin_audio_stream");
                break;
            case Moving_clapping_270_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180819]_moving_source_conti_clap_3/origin_audio_stream");
                break;
            case Moving_voice_270_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180819]_moving_source_voice_2/origin_audio_stream");
                break;
            case Moving_voice_Obs_155_140_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180909]_moving_source_clapping_obs_140_155_0_v1/origin_audio_stream");
                break;
            case Moving_voice_Obs_155_110_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180909]_moving_source_clapping_obs_140_110_0_v3/origin_audio_stream");
                break;

            }
        }

        std::string getSourcePosFilePath(int idx){

            std::string projectPath("/home/inkyu/catkin_ssl_ws/src/tdoa_based_SSL/localization");
            switch(idx){
            case Moving_clapping_270_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180819]_moving_source_conti_clap_3/origin_robot_pos");
                break;
            case Moving_voice_270_0_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180819]_moving_source_voice_2/origin_robot_pos");
                break;
            case Moving_voice_Obs_155_140_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180909]_moving_source_clapping_obs_140_155_0_v1/origin_robot_pos");
                break;
            case Moving_voice_Obs_155_110_0:
                return projectPath + std::string("/data/raw_data/raw_data_moving_sound/[180909]_moving_source_clapping_obs_140_110_0_v3/origin_robot_pos");
                break;
            default:
                return std::string("");
                break;
            }
        }

        void getGroundTruthPosition(int idx, double& x, double& y, double& z){
            double source_height = 0.1;
            switch(idx){
            case Clapping_290_0_0:
                x = 2.9f;
                y = 0.0f;
                z = source_height;
                break;
            case Voice_290_0_0:
                x = 2.9f;
                y = 0.0f;
                z = source_height;
                break;
            case Clapping_290_55_0:
                x = 2.9f;
                y = -0.55f;
                z = source_height;
                break;
            case Voice_290_55_0:
                x = 2.9f;
                y = -0.55f;
                z = source_height;
                break;
            case Clapping_290_110_0:
                x = 2.9f;
                y = -1.10f;
                z = source_height;
                break;
            case Voice_290_110_0:
                x = 2.9f;
                y = -1.10f;
                z = source_height;
                break;
            case Clapping_inv_200_0_0:
                x = -2.0f;
                y = 0.0f;
                z = source_height;
                break;
            case Voice_inv_200_0_0:
                x = -2.0f;
                y = 0.0f;
                z = source_height;
                break;
            case IR_270_0_30:
                x = 2.7f;
                y = 0.0f;
                z = 0.3f;
                break;
            }
        }

        //ROS variables
        ros::NodeHandle local_nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        //ros::Publisher pubSceneState;
        ros::Publisher pubSourcePos;
        ros::Timer timer_;
        //topic_filters::filtered_publisher pub_;
        ros::Time ros_init_time_;        
        bool is_audio_saved_;
        //bool use_saved_audio;
        bool enable_separation_;
        std::string frame_id_;
        bool instant_time_;
        std::string iterative_path_;
        int iterative_delay_;
        bool iterative_enable_;
        int system_delay_;
        int scene_state_;
        bool save_result_;

        // Output gain
        double gain_sep_;
        double gain_pf_;

        //Manyears variables
        unsigned int frame_number_;
        short audio_raw_data_[manyears_global::raw_buffer_size_s];
        double source_position[3];
        float audio_float_data_[manyears_global::nb_microphones_s][manyears_global::samples_per_frame_s];
        FILE* filePtr_;             // File pointer for reading Raw audio stream data of all microphones
        FILE* filePtr_sourcePos;             //
        FILE* output_file_[8];      // File pointer for writing Raw audio stream data of each Microphone
        FILE* output_file_origin;   // File pointer for writing Raw audio stream data of all microphones
        FILE* output_file_robot_pos;    // File pointer for writing pobot position data

        // TF listener
        tf::TransformListener tfListener;

        struct objOverall libraryContext_;
        unsigned int sequence;

        // ray based
        ros::Time ros_audio_get_time_;
    };
}

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "tdoa_ra_ssl_node");
    ros::NodeHandle n;

    ros::Duration(3.0).sleep();

    sound_localization_node::sound_localization SL_node(n);

    ros::spin();


    SL_node.terminate();
    return 0;
}
