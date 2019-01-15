#pragma once

#include <vision/ObjectDetector.h>
#include <vision/ColorSegmenter.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DIFF_RATIO_THRESHOLD 8
#define DIFF_BALL_THRESHOLD 20

enum PenaltyKickKeeperState {IS_MOVING, FIND_BALL, WATCH_BALL_AND_FIND_ROBOT, BALL_MOVEMENT_DETECTED};

/// @ingroup vision
class PenaltyKeeperImageProcessor : public ObjectDetector {
	public:
    	PenaltyKeeperImageProcessor(DETECTOR_DECLARE_ARGS, ColorSegmenter& segmenter);
		bool penaltyKeeperProcessFrame();

	private:

	    ColorSegmenter& color_segmenter_;

		PenaltyKickKeeperState state_;

		const int threshold_ = 20;
		const int window_ = 7;

		const int xstep_ = (1 << 3);
		const int ystep_ = (1 << 2);

		// const int wholeXmin_ = 0;
		// const int wholeXmax_ = 1280;
		// const int wholeYmin_ = 0;
		// const int wholeYmax_ = 960;
		const int wholeXmin_ = 300;
		const int wholeXmax_ = 880;
		const int wholeYmin_ = 280;
		const int wholeYmax_ = 580;
		const int wholeRectWidth_ = wholeXmax_ - wholeXmin_;
		const int wholeRectHeight_ = wholeYmax_ - wholeYmin_;

		cv::Rect wholeRectLarge_;
		cv::Rect ballRectLarge_;
		cv::Rect belowRectLarge_;

		cv::Mat prevBallMatSmall_;
		cv::Mat prevBelowMatSmall_;

		std::vector<cv::Rect> ballRectHistory_;

		cv::Rect large2small(cv::Rect& r);
		cv::Rect small2large(cv::Rect& r);
		cv::Point findBottomPoint(cv::Mat& mat);
		bool enoughBallEvidence(cv::Rect& avgRect);
		void findBall();
		bool ballMoved();
		void findRobot();
		void resetVariables();
		bool pointInRect(int y, int x, cv::Rect& r);
		void alignRect(cv::Rect& rect, int xstep, int ystep);
	  	inline void alignRect(cv::Rect& rect) {alignRect(rect, xstep_, ystep_);}
		int topFrameCounter_ = 0;

#ifdef TOOL
		char* user_ = "jkelle";
#else
		char* user_ = "nao";
#endif
};
