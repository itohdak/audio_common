#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "audio_common_msgs/AudioData.h"

namespace audio_transport
{
  class RosGstPlay
  {
    public:
      RosGstPlay()
      {
        GstPad *audiopad;

        std::string dst_type;
        std::string device;

        // The destination of the audio
        ros::param::param<std::string>("~dst", dst_type, "alsasink");
        ros::param::param<std::string>("~device", device, std::string());

        std::string _input_format;
        std::string _output_format;

        // Need to encoding or publish raw wave data
        ros::param::param<std::string>("~input_format", _input_format, "mp3");
        ros::param::param<std::string>("~output_format", _output_format, "mp3");

        // The bitrate at which to encode the audio
        ros::param::param<int>("~bitrate", _bitrate, 192);

        // only available for raw data
        ros::param::param<int>("~channels", _channels, 1);
        ros::param::param<int>("~depth", _depth, 16);
        ros::param::param<int>("~sample_rate", _sample_rate, 16000);

        _sub = _nh.subscribe("audio", 10, &RosGstPlay::onAudio, this);

        _loop = g_main_loop_new(NULL, false);

        _pipeline = gst_pipeline_new("app_pipeline");
        _source = gst_element_factory_make("appsrc", "app_source");
        g_object_set(G_OBJECT(_source), "do-timestamp", TRUE, NULL);
        gst_bin_add( GST_BIN(_pipeline), _source);

        //_playbin = gst_element_factory_make("playbin2", "uri_play");
        //g_object_set( G_OBJECT(_playbin), "uri", "file:///home/test/test.mp3", NULL);
        if (dst_type == "alsasink")
        {
          _decoder = gst_element_factory_make("decodebin", "decoder");
          g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);
          gst_bin_add( GST_BIN(_pipeline), _decoder);
          gst_element_link(_source, _decoder);

          _audio = gst_bin_new("audiobin");
          _convert = gst_element_factory_make("audioconvert", "convert");
          audiopad = gst_element_get_static_pad(_convert, "sink");
          _sink = gst_element_factory_make("autoaudiosink", "sink");
          if (!device.empty()) {
            g_object_set(G_OBJECT(_sink), "device", device.c_str(), NULL);
          }
          gst_bin_add_many( GST_BIN(_audio), _convert, _sink, NULL);
          gst_element_link(_convert, _sink);
          gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
          gst_object_unref(audiopad);

          gst_bin_add(GST_BIN(_pipeline), _audio);
        }
        else
        {
          _sink = gst_element_factory_make("filesink", "sink");
          g_object_set( G_OBJECT(_sink), "location", dst_type.c_str(), NULL);
          /*---------------------------------------------------------------------*/
          if (_input_format == "wave" && _output_format == "mp3") {
            // parser
            _filter = gst_element_factory_make("wavparse", "wav-parser");
            // g_signal_connect(_filter, "pad-added", G_CALLBACK(cb_newpad), _sink);
            g_signal_connect(_filter, "pad-added", G_CALLBACK(cb_newpad), this);
            // gst_bin_add(GST_BIN(_pipeline), _filter);
            // gst_element_link(_source, _filter);

            // converter
            _convert = gst_element_factory_make("audioconvert", "convert");
            if (!_convert) {
              ROS_ERROR_STREAM("Failed to create audioconvert element");
              exit(1);
            }
            gst_bin_add(GST_BIN(_pipeline), _convert);
            // gst_element_link(_filter, _convert);
            std::cout << gst_element_link(_source, _convert) << std::endl;

            // encoder
            if (_output_format == "wave") {
              _encoder = gst_element_factory_make("wavenc", "encoder");
            } else if (_output_format == "mp3") {
              _encoder = gst_element_factory_make("lamemp3enc", "encoder");
              g_object_set( G_OBJECT(_encoder), "quality", 2.0, NULL);
              g_object_set( G_OBJECT(_encoder), "bitrate", _bitrate, NULL);
            }
            if (!_encoder) {
        	  ROS_ERROR_STREAM("Failed to create encoder element");
        	  exit(1);
            }
            gst_bin_add(GST_BIN(_pipeline), _encoder);
            std::cout << gst_element_link(_convert, _encoder) << std::endl;

            gst_bin_add(GST_BIN(_pipeline), _sink);
            std::cout << gst_element_link(_encoder, _sink) << std::endl;
          }
          /*---------------------------------------------------------------------*/
          else if (_input_format == "mp3" && _output_format == "wave") {
            // decoder
            _decoder = gst_element_factory_make("decodebin", "decoder");
            g_signal_connect(_decoder, "pad-added", G_CALLBACK(cb_newpad),this);
            gst_bin_add( GST_BIN(_pipeline), _decoder);
            if (gst_element_link(_source, _decoder) != TRUE)
              {
                std::cout << "Error occured in linking decoder \n";
                exit(1);
              }

            _audio = gst_bin_new("audiobin");
            // converter
            _convert = gst_element_factory_make("audioconvert", "convert");
            // encoder
            _encoder = gst_element_factory_make("wavenc", "encoder");
            audiopad = gst_element_get_static_pad(_convert, "sink");

            gst_bin_add_many( GST_BIN(_audio), _convert, _encoder, _sink, NULL);
            if (gst_element_link_many (_convert, _encoder, _sink, NULL) != TRUE)
              {
                std::cout << "Error occured in linking audio converter, encoder and sink \n";
                exit(1);
              }

            gst_element_add_pad(_audio, gst_ghost_pad_new("sink", audiopad));
            gst_object_unref(audiopad);

            gst_bin_add(GST_BIN(_pipeline), _audio);
          }
          else if (_input_format == "mp3" && _output_format == "mp3")
          {
            gst_bin_add(GST_BIN(_pipeline), _sink);
            gst_element_link(_source, _sink);
          }
          else
          {
            ROS_ERROR_STREAM("Unsupported media type.");
            exit(1);
          }
        }

        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
        //gst_element_set_state(GST_ELEMENT(_playbin), GST_STATE_PLAYING);

        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
      }

    private:

      void onAudio(const audio_common_msgs::AudioDataConstPtr &msg)
      {
        GstBuffer *buffer = gst_buffer_new_and_alloc(msg->data.size());
        gst_buffer_fill(buffer, 0, &msg->data[0], msg->data.size());
        GstFlowReturn ret;

        g_signal_emit_by_name(_source, "push-buffer", buffer, &ret);
      }

     static void cb_newpad (GstElement *decodebin, GstPad *pad, 
                             gpointer data)
      {
        RosGstPlay *client = reinterpret_cast<RosGstPlay*>(data);

        GstCaps *caps;
        GstStructure *str;
        GstPad *audiopad;

        /* only link once */
        audiopad = gst_element_get_static_pad (client->_audio, "sink");
        if (GST_PAD_IS_LINKED (audiopad)) 
        {
          g_object_unref (audiopad);
          return;
        }

        /* check media type */
        caps = gst_pad_query_caps (pad, NULL);
        str = gst_caps_get_structure (caps, 0);
        if (!g_strrstr (gst_structure_get_name (str), "audio")) {
          gst_caps_unref (caps);
          gst_object_unref (audiopad);
          return;
        }

        gst_caps_unref (caps);

        /* link'n'play */
        gst_pad_link (pad, audiopad);

        g_object_unref (audiopad);
      }

      ros::NodeHandle _nh;
      ros::Subscriber _sub;
      boost::thread _gst_thread;

      GstElement *_pipeline, *_source, *_sink, *_decoder, *_convert, *_audio, *_encoder, *_filter;
      GstElement *_playbin;
      GMainLoop *_loop;
      int _bitrate, _channels, _depth, _sample_rate;
  };
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "audio_play");
  gst_init(&argc, &argv);

  audio_transport::RosGstPlay client;

  ros::spin();
}
