#include "TouchDetection.h"
#include "DLibInterface.h"
#include "Util.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


using namespace std;
using namespace cv;
using namespace TouchDetectionUtils;

TouchDetection::TouchDetection():m_initializedBackground(false),
                                 m_pixelHistogramFrames(0),
                                 m_two_hands(false),
                                 m_last_two_hands(false)
{

}

TouchDetection::~TouchDetection() {

}


std::vector<Touches> TouchDetection::process(const cv::Mat_<float>& depth) {
    //process the depth image and detect touches
    //at first need to read a few frames to build the BG model...
    std::vector<Touches> res;
    if (not m_initializedBackground) {
        if (m_touch_mask.empty()) {
            m_touch_mask = Mat(depth.size(), CV_8UC1, Scalar(255));
        }

        if (m_pixelDepthHistogram.empty() or m_pixelHistogramFrames == 0) {
            cout << "init pixel histograms: " << flush;

            initializePixelHistograms(depth);
            m_pixelHistogramFrames = 1;
        } else {
            cout << "." << flush;

            addFrameToPixelHistograms(depth);
            m_pixelHistogramFrames++;

            if (m_pixelHistogramFrames == m_maxPixelHistogramFrames) {
                cout << "Done." << endl;

                findDMaxDMinFromPixelHistogram();

                //mask for hand contours
                m_hand_min_thresh = (m_dhandmin > 0) & m_touch_mask;

                //mask for finger contours
                m_finger_min_thresh = (m_dmax > 0) & m_touch_mask;

                m_initializedBackground = true;
            }
        }

        res.push_back(Touches());
        return res;
    }

    //Find the hand mask, to prepare for hand pose estimation
    Mat handMaskRaw = (depth < m_dmax) & (depth > m_dhandmin); // & m_hand_min_thresh;
    Mat handMask;
    handMaskRaw.convertTo(handMask, CV_32FC1);
    boxFilter(handMask, handMask, -1, Size(9, 9));
    handMask = handMask > 128.0 & handMaskRaw;


    // Mat_<Vec3i> rdfSegmentation = m_rdf.process(depth);
    // Mat rdfSegOut; rdfSegmentation.convertTo(rdfSegOut, CV_8UC3);
    // std::vector<Mat> v(3);
    // split(rdfSegOut, v);
    //
    // imshow("rdf", rdfSegOut);
    //
    // //get the RDF arm mask
    // Mat armMask = (v[1] == 255);
    // boxFilter(armMask, armMask, -1, Size(9, 9));
    // armMask = armMask > 128.0;
    //
    // //the hand is given by subtarcting the arm from the foreground blob
    // handMask = handMask & ~armMask;

    std::vector<std::vector<Point> > handContours;
    findContours(handMask, handContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //TODO instead of taking the largest blob - analyze each valid blob for touch points

    // get hand largest contour
    int largestHandContourIdx = -1;
    int largestContourArea = -1;
    for (int i = 0; i < handContours.size(); i++) {
        const int candidateContourArea = contourArea(handContours[i]);
        cout << candidateContourArea << "!!!!!!!!!!!!!!" << endl;
        if (candidateContourArea > m_min_hand_contour_area and
            candidateContourArea < m_max_hand_contour_area and
            candidateContourArea > largestContourArea)
        {
            largestHandContourIdx = i;
            largestContourArea = candidateContourArea;
        }
    }

    //TODO instead of taking the largest blob - take the two largest bolbs
    int secondLargestHandContourIdx = -1;
    int secondLargestHandContourArea = -1;
    for(int i = 0; i < handContours.size(); i++) {
        const int candidateContourArea = contourArea(handContours[i]);
        if(candidateContourArea > m_min_hand_contour_area and
            candidateContourArea < m_max_hand_contour_area and
            candidateContourArea > secondLargestHandContourArea and
            i != largestHandContourIdx) {

            secondLargestHandContourIdx = i;
            secondLargestHandContourArea = candidateContourArea;
        }
    }

    //no valid hand contours found
    if (largestHandContourIdx == -1) {
        res.push_back(Touches());
        m_last_two_hands = m_two_hands;
        return res;
    }
    if(secondLargestHandContourIdx != -1) {
        m_two_hands = true;
    }
    else {
        m_two_hands = false;
    }

    //take just the hand ROI
    Rect handROI = boundingRect(handContours[largestHandContourIdx]);
    Rect handROIExtra;
    if(m_two_hands) {
        handROIExtra = boundingRect(handContours[secondLargestHandContourIdx]);
    }

    //detect touches by subtarcting dmax and contour analysis
    Mat touchMask = (depth < m_dmax) & (depth > m_dfingermin) & m_finger_min_thresh;
    touchMask.convertTo(touchMask, CV_32FC1);
    boxFilter(touchMask, touchMask, -1, Size(7, 7));
    touchMask = touchMask > 128.0;
    // imshow("touchMask", touchMask);
    if(cv::sum(touchMask)[0] > m_max_touch_mask) {
        res.push_back(Touches());
        m_last_two_hands = m_two_hands;
        return res;
    }

    std::vector<std::vector<Point> > touchContours;
    std::vector<std::vector<Point> > touchContoursExtra;
    findContours(Mat(handMask(handROI) & touchMask(handROI)), touchContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(m_two_hands) {
        findContours(Mat(handMask(handROIExtra) & touchMask(handROIExtra)), touchContoursExtra, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    }
    Mat m1 = Mat(handMask(handROI) & touchMask(handROI));
    imshow("m1", m1);
    if(m_two_hands) {
        Mat m2 = Mat(handMask(handROIExtra) & touchMask(handROIExtra));
        imshow("m2", m2);
    }

    Touches touches;
    Touches touchesExtra;
    const Rect imageRect(2, 2, depth.cols - 4, depth.rows - 4);
    for(const auto& c : touchContours) {
        if (contourArea(c) < 25.0 || contourArea(c) >= 180.0) {
            continue;
        }

        Scalar midp = mean(Mat(c));
        const Point2f roiTL = handROI.tl();
        const Point2f tipPoint = Point2f(midp.val[0], midp.val[1]) + roiTL;

        if (not imageRect.contains(tipPoint)) {
            continue;
        }

        std::vector<Point> newc(c.size());
        //translate the contour to global image coordinates according to ROI
        Mat(Mat(c) + Scalar(roiTL.x, roiTL.y)).copyTo(Mat(newc));

        // touch filtering according to [Harrison 2016]
        //if (not isHovering(depth, tipPoint)) {
            //translate tip point from ROI
            touches.push_back({ tipPoint, tipPoint, newc, FingerCode::NONE, TouchState::UP, 0});
        //}
    }

    if(m_two_hands) {
        for(const auto& c : touchContoursExtra) {
            if (contourArea(c) < 25.0 || contourArea(c) >= 180.0) {
                continue;
            }

            Scalar midp = mean(Mat(c));
            const Point2f roiTL = handROIExtra.tl();
            const Point2f tipPoint = Point2f(midp.val[0], midp.val[1]) + roiTL;

            if (not imageRect.contains(tipPoint)) {
                continue;
            }

            std::vector<Point> newc(c.size());
            //translate the contour to global image coordinates according to ROI
            Mat(Mat(c) + Scalar(roiTL.x, roiTL.y)).copyTo(Mat(newc));

            //if(not isHovering(depth, tipPoint)) {
                touchesExtra.push_back({ tipPoint, tipPoint, newc, FingerCode::NONE, TouchState::UP, 0});
            //}
        }
    }

    // if(m_two_hands) {
    //     if(touches.empty() && touchesExtra.empty()) {
    //         m_lastTouches = toTouchForFinger(touches);
    //         m_lastTouchesExtra = toTouchForFinger(touchesExtra);
    //         res.push_back(touches);
    //         res.push_back(touchesExtra);
    //         m_last_two_hands = m_two_hands;
    //         return res;
    //     }
    // }
    // else {
    //     if(touches.empty()) {
    //         m_lastTouches = toTouchForFinger(touches);
    //         res.push_back(touches);
    //         m_last_two_hands = m_two_hands;
    //         return res;
    //     }
    // }

    // if (touches.size() == 0) {
    //     m_lastTouches.clear();
    //     res.push_back(touches);
    //     return res;
    // }

    //hand 1
    Mat handMask1, handMask1temp;
    handMask.copyTo(handMask1);
    handMask.copyTo(handMask1temp);
    handMask1temp.setTo(0);
    handMask1temp(handROI) = Scalar(255, 255, 255);
    handMask1.setTo(0, ~handMask1temp);
    //hand2
    Mat handMask2, handMask2temp;
    if(m_two_hands) {
        handMask.copyTo(handMask2);
        handMask.copyTo(handMask2temp);
        handMask2temp.setTo(0);
        handMask2temp(handROIExtra) = Scalar(255, 255, 255);
        handMask2.setTo(0, ~handMask2temp);
    }
    // Get all the joints then use them to get the fingertips
    imshow("hand", handMask1);
    cv::waitKey(1);
    FingerTips fingerTips1 = m_hand_pose_estimator.process(depth, handMask1);
    FingerTips fingerTips2;
    Mat dis = depth / 1000;
    dis.setTo(2.0, ~handMask1);
    cv::cvtColor(dis, dis, CV_GRAY2BGR);
    if(m_two_hands) {
        fingerTips2 = m_hand_pose_estimator.process(depth, handMask2);
        Mat tt = depth / 1000;
        tt.setTo(2.0, ~handMask2);
        cv::cvtColor(tt, tt, CV_GRAY2BGR);
        dis = dis | tt;
    }
    drawFingertips(dis, fingerTips1);
    if(m_two_hands) {
        drawFingertips(dis, fingerTips2);
    }
    imshow("all", dis);

    // if (fingerTips1.size() == 0) {
    //     res.push_back(touches);
    //     m_lastTouches = toTouchForFinger(touches);
    //     m_last_two_hands = m_two_hands;
    //     return res;
    // }

    //split touches to multiple fingers based on the detected fingerTips.
    touches = splitTouchBlobs(touches);
    //touches = splitTouchBlobsNew(touches, fingerTips1);
    if(m_two_hands) {
        touchesExtra = splitTouchBlobs(touchesExtra);
        //touchesExtra = splitTouchBlobsNew(touchesExtra, fingerTips2);
    }

    //match the fingerTips to the touch points
    //this way we get the finger index (thumb, middle, ...) for each touch
    //point.
    touches = fingerDesignation(touches, fingerTips1);
    if(m_two_hands) {
        touchesExtra = fingerDesignation(touchesExtra, fingerTips2);
    }

    //update the TouchState of the touches
    if(m_last_two_hands) {
        touches = matchTouches12(touches, m_lastTouches, m_lastTouchesExtra);
        if(m_two_hands) {
            touchesExtra = matchTouches12(touchesExtra, m_lastTouches, m_lastTouchesExtra);
        }
    }
    else {
        if(m_two_hands) {
            matchTouches21(touches, touchesExtra, m_lastTouches);
        }
        else {
            touches = matchTouches11(touches, m_lastTouches);
        }
    }

    //TODO  filter touches with Kalman Filter
    //touches = filterTouches(touches);

    m_lastTouches = toTouchForFinger(touches);
    if(m_two_hands) {
        m_lastTouchesExtra = toTouchForFinger(touchesExtra);
    }

    m_last_two_hands = m_two_hands;

    
    res.push_back(touches);
    if(m_two_hands) {
        res.push_back(touchesExtra);
    }

    return res;
}


void TouchDetection::drawFingertips(const Mat& depth, const FingerTips& tips) {
    for (const FingerTip& t : tips) {
        const Scalar color = HandPoseUtils::colorWheel[t.code];
        circle(depth, t.tipPoint, 3, color, CV_FILLED);
        switch (t.code) {
            case THUMB:  putText(depth, "THM", t.tipPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
            case INDEX:  putText(depth, "IDX", t.tipPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
            case MIDDLE: putText(depth, "MID", t.tipPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
            case RING:   putText(depth, "RNG", t.tipPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
            case LITTLE: putText(depth, "LIT", t.tipPoint, FONT_HERSHEY_SIMPLEX, 0.5, color); break;
            default:     putText(depth, "UNK", t.tipPoint, FONT_HERSHEY_SIMPLEX, 0.5, color);
        }
    }
}

Touches TouchDetection::fixMatching(const Touches& touches, const Touches& lastTouches) {
    const float matchDistanceThresh = 30;
    const float bigPenalty = 10000; // add threshold for match, the match should be within a maximum distance

    Mat_<int> matchMatrix(std::min(5, (int)touches.size()), lastTouches.size());
    for (size_t i = 0; i < matchMatrix.rows; i++) {
        const Touch& touch = touches[i];
        for (size_t j = 0; j < matchMatrix.cols; j++) {
            const double n = norm(lastTouches[j].touchPoint - touch.touchPoint);
            if (n > matchDistanceThresh) {
                matchMatrix(i, j) = -bigPenalty;
                continue;
            }
            matchMatrix(i, j) = -cvRound(n * n);
        }
    }

    //find optimal match
    std::vector<long> match = DLibInterface::linearMatching(matchMatrix);
    Touches res = touches;
    for (size_t i = 0; i < std::min(res.size(), match.size()); i++) {
        if(norm(res[i].touchPoint - lastTouches[match[i]].touchPoint) < m_diff_thresh) {
            res[i].finger = lastTouches[match[i]].finger;
            //cout << match[0] << " " << match[1] << " " << match[2] << endl;
        }
    }
    return res;
}

Touches TouchDetection::matchTouches11(const Touches& touches, const Touches& lastTouches) {
    //match last frame touches to current
    //build a cost matrix
    if(touches.empty()) {
        return touches;
    }
    Touches lastTouchesThis = toTouches(lastTouches);
    if(lastTouchesThis.empty()) {
        Touches res = touches;
        for(Touch& t : res) {
            t.state = DOWN;
        }
        return res;
    }

    //run Hungarian matching algorithm (DLibInterface::linearMatching)

    //update state for new touches
    //if found new touch in last frame - TouchState = DRAG

    //if not found new touch in last frame - TouchState = DOWN

    //if not found any match for last frame touch - TouchState = UP

    //keep track of recent history for the new touch (matched, i.e. a DRAG touch)

    //give "grace period" for UP touches (hysteresis):
    //count the number of frames this touch was detected, with maximum count of 5.
    //if the touch was not detected, subtract 1 from this count.
    //if the count reaches 0 - consider the touch to be UP.
    Touches fixedTouches = fixMatching(touches, lastTouchesThis);

    TouchForFinger currentTouches = toTouchForFinger(fixedTouches);

    for(int i = 0; i < currentTouches.size(); i++) {
        //down
        if(currentTouches[i].finger != NONE && lastTouches[i].state == UP) {
            // if(currentTouches[i].count == m_max_touch_count) {
            //     currentTouches[i].state = DOWN;
            //     currentTouches[i].count = 0;
            // }
            // else {
            //     currentTouches[i].count += lastTouches[i].count;
            //     currentTouches[i].count ++;
            // }
            currentTouches[i].state = DOWN;
        }
        else if(currentTouches[i].finger != NONE && currentTouches[i].finger == lastTouches[i].finger && lastTouches[i].state != UP) {
            const Point2f currentPoint = currentTouches[i].touchPoint;
            const Point2f lastPoint = lastTouches[i].touchPoint;
            const float diff = norm(currentPoint - lastPoint);

            //drag
            if(diff > m_diff_thresh) {
                currentTouches[i].state = DRAG;
                }
            //hold
            else {
                currentTouches[i].state = HOLD;
            }
        }
        else if(currentTouches[i].finger == NONE && currentTouches[i].state != UP) {
            //up
            if(currentTouches[i].count == m_max_touch_count) {
                currentTouches[i].state = UP;
                currentTouches[i].count = 0;
            }
            else {
                currentTouches[i].count += lastTouches[i].count;
                currentTouches[i].count ++;
            }
        }
    }

    Touches res;
    for(const Touch& t : currentTouches) {
        if(t.finger != NONE) {
            res.push_back(t);
        }
    }

    return res;
}

Touches TouchDetection::matchTouches12(const Touches& touches, const Touches& lastTouches, const Touches& lastTouchesExtra) {
    Touches lastTouchesThis = toTouches(lastTouches);
    Touches lastTouchesExtraThis = toTouches(lastTouchesExtra);
    if(lastTouchesThis.empty()) {
        return matchTouches11(touches, lastTouchesExtra);
    }
    if(lastTouchesExtraThis.empty()) {
        return matchTouches11(touches, lastTouches);
    }

    const float matchDistanceThresh = 30;
    const float bigPenalty = 10000; // add threshold for match, the match should be within a maximum distance

    Mat_<int> matchMatrix1(std::min(5, (int)touches.size()), lastTouchesThis.size());
    for (size_t i = 0; i < matchMatrix1.rows; i++) {
        const Touch& touch = touches[i];
        for (size_t j = 0; j < matchMatrix1.cols; j++) {
            const double n = norm(lastTouchesThis[j].touchPoint - touch.touchPoint);
            if (n > matchDistanceThresh) {
                matchMatrix1(i, j) = -bigPenalty;
                continue;
            }
            matchMatrix1(i, j) = -cvRound(n * n);
        }
    }

    //find optimal match
    std::vector<long> match1 = DLibInterface::linearMatching(matchMatrix1);

    float totalDistance1 = 0.0;
    for (size_t i = 0; i < std::min(touches.size(), match1.size()); i++) {
        totalDistance1 += matchMatrix1(i, match1[i]);
    }

    Mat_<int> matchMatrix2(std::min(5, (int)touches.size()), lastTouchesExtraThis.size());
    for (size_t i = 0; i < matchMatrix2.rows; i++) {
        const Touch& touch = touches[i];
        for (size_t j = 0; j < matchMatrix2.cols; j++) {
            const double n = norm(lastTouchesExtraThis[j].touchPoint - touch.touchPoint);
            if (n > matchDistanceThresh) {
                matchMatrix2(i, j) = -bigPenalty;
                continue;
            }
            matchMatrix2(i, j) = -cvRound(n * n);
        }
    }

    //find optimal match
    std::vector<long> match2 = DLibInterface::linearMatching(matchMatrix2);

    float totalDistance2 = 0.0;
    for (size_t i = 0; i < std::min(touches.size(), match2.size()); i++) {
        totalDistance2 += matchMatrix2(i, match2[i]);
    }

    Touches matchedLastTouches;
    if(totalDistance1 > totalDistance2) {
        matchedLastTouches = lastTouches;
    }
    else {
        matchedLastTouches = lastTouchesExtra;
    }
    return matchTouches11(touches, matchedLastTouches);
}


void TouchDetection::matchTouches21(Touches& touches, Touches& touchesExtra, const Touches& lastTouches) {
    Touches lastTouchesThis = toTouches(lastTouches);
    if(lastTouchesThis.empty()) {
        return;
    }
    if(touches.empty()) {
        touchesExtra = matchTouches11(touchesExtra, lastTouches);
        return;
    }
    if(touchesExtra.empty()) {
        touches = matchTouches11(touches, lastTouches);
        return;
    }

    const float matchDistanceThresh = 30;
    const float bigPenalty = 10000; // add threshold for match, the match should be within a maximum distance

    Mat_<int> matchMatrix1(lastTouchesThis.size(), std::min(5, (int)touches.size()));
    for (size_t i = 0; i < matchMatrix1.rows; i++) {
        const Touch& touch = lastTouchesThis[i];
        for (size_t j = 0; j < matchMatrix1.cols; j++) {
            const double n = norm(touches[j].touchPoint - touch.touchPoint);
            if (n > matchDistanceThresh) {
                matchMatrix1(i, j) = -bigPenalty;
                continue;
            }
            matchMatrix1(i, j) = -cvRound(n * n);
        }
    }

    //find optimal match
    std::vector<long> match1 = DLibInterface::linearMatching(matchMatrix1);

    float totalDistance1 = 0.0;
    for (size_t i = 0; i < std::min(lastTouchesThis.size(), match1.size()); i++) {
        totalDistance1 += matchMatrix1(i, match1[i]);
    }

    Mat_<int> matchMatrix2(lastTouchesThis.size(), std::min(5, (int)touchesExtra.size()));
    for (size_t i = 0; i < matchMatrix2.rows; i++) {
        const Touch& touch = lastTouchesThis[i];
        for (size_t j = 0; j < matchMatrix2.cols; j++) {
            const double n = norm(touchesExtra[j].touchPoint - touch.touchPoint);
            if (n > matchDistanceThresh) {
                matchMatrix2(i, j) = -bigPenalty;
                continue;
            }
            matchMatrix2(i, j) = -cvRound(n * n);
        }
    }

    //find optimal match
    std::vector<long> match2 = DLibInterface::linearMatching(matchMatrix2);

    float totalDistance2 = 0.0;
    for (size_t i = 0; i < std::min(lastTouchesThis.size(), match2.size()); i++) {
        totalDistance2 += matchMatrix2(i, match2[i]);
    }

    if(totalDistance1 > totalDistance2) {
        touches = matchTouches11(touches, lastTouches);
        return;
    }
    else {
        touchesExtra = matchTouches11(touchesExtra, lastTouches);
        return;
    }
}


TouchForFinger TouchDetection::toTouchForFinger(const Touches& touches) {
    TouchForFinger touchesForFingers(6);
    for (const Touch& t : touches) {
        touchesForFingers[t.finger] = t;
    }
    return touchesForFingers;
}

Touches TouchDetection::toTouches(const TouchForFinger& touchesForFingers) {
    Touches touches;
    for (const Touch& t : touchesForFingers) {
        if(t.finger != NONE) {
            touches.push_back(t);
        }
    }
    return touches;
}

Touches TouchDetection::fingerDesignation(const Touches& touches, const FingerTips& fingerTips) {
    // if(touches.size() == 0) {
    //     return touches;
    // }
    //set up touch-finger cost matrix (based on distance)
    const float matchDistanceThresh = 30; // add threshold for match, the match should be within a maximum distance
    const float bigPenalty = 10000; // add threshold for match, the match should be within a maximum distance

    Mat_<int> matchMatrix(std::min(5, (int)touches.size()), fingerTips.size());
    for (size_t i = 0; i < matchMatrix.rows; i++) {
        const Touch& touch = touches[i];
        for (size_t j = 0; j < matchMatrix.cols; j++) {
            const double n = norm(fingerTips[j].tipPoint - touch.touchPoint);
            if (n > matchDistanceThresh) {
                matchMatrix(i, j) = -bigPenalty;
                continue;
            }
            //add to the cost some temporal information, to keep finger assignment consistant
            // double distanceToLast = 0.0;
            // if (not m_lastTouches.empty()) {
            //     distanceToLast = norm(m_lastTouches[fingerTips[j].code].touchPoint - touch.touchPoint);
            // }
            // matchMatrix(i, j) = -cvRound(n * n + distanceToLast);
            matchMatrix(i, j) = -cvRound(n * n);
        }
    }



    // Mat_<int> matchMatrix2;
    // std::vector<long> match2;
    // double prevTouchThresh = 30;
    // if (not m_lastTouches.empty()) {
    //     matchMatrix2 = Mat_<int>(std::min(5, (int)touches.size()), m_lastTouches.size());
    //     for (size_t i = 0; i < matchMatrix2.rows; i++) {
    //         const Touch& touch = touches[i];
    //         for (size_t j = 0; j < matchMatrix2.cols; j++) {
    //             const double n = norm(m_lastTouches[j].touchPoint - touch.touchPoint);
    //             matchMatrix2(i, j) = -cvRound(n * n );
    //         }
    //     }
    //     match2 = DLibInterface::linearMatching(matchMatrix2);
    // }

    //find optimal match
    std::vector<long> match = DLibInterface::linearMatching(matchMatrix);

    //assign finger designation
    Touches touchesWithFinegrs = touches;
    for (size_t i = 0; i < std::min(touches.size(), match.size()); i++) {
        if (matchMatrix(i,match[i]) <= -bigPenalty)
            continue;
        // if (not m_lastTouches.empty() 
        //     && matchMatrix2(i,match2[i]) >= -prevTouchThresh
        //     && fingerTips[match[i]].code != m_lastTouches[match2[i]].finger){
        //     touchesWithFinegrs[i].finger             = fm_lastTouches[match2[i]].finger;
        //     touchesWithFinegrs[i].estimatedFingerTip = fingerTips[match2[i]].tipPoint;

        // }
        // else{

            // if (matchMatrix(i,match[i]) < -matchDistanceThresh)
            //     continue;
            touchesWithFinegrs[i].finger             = fingerTips[match[i]].code;
            touchesWithFinegrs[i].estimatedFingerTip = fingerTips[match[i]].tipPoint;
        // }
    }

    return touchesWithFinegrs;
}

Touches TouchDetection::splitTouchBlobsNew(const Touches& touches, const FingerTips& fingerTips) {
    Touches splitTouches;
    for(int i = 0; i < touches.size(); i++) {
        bool found = false;
        for(int j = 0; j < fingerTips.size(); j++) {
            double dis = cv::pointPolygonTest(touches[i].contour, fingerTips[j].tipPoint, true);
            if(dis > -5.0) {
                splitTouches.push_back({ fingerTips[j].tipPoint, fingerTips[j].tipPoint, touches[i].contour, FingerCode::NONE , TouchState::UP, 0});
                found = true;
            }
        }
        if(not found) {
            splitTouches.push_back(touches[i]);
        }
    }

    return splitTouches;
}

Touches TouchDetection::splitTouchBlobs(const Touches& touches) {
    Touches splitTouches;
    const int sampleRate      = 9;
    const int twiceSampleRate = sampleRate * 2;
    const int midSampleCoord  = (int)(sampleRate / 2.0 + 0.5);
    const int smallStep       = 3;
    const int bigStep         = 9;

    for (int i = 0; i < touches.size(); i++) {
        bool found = false;
        Rect contourRect = boundingRect(touches[i].contour);

        if (contourRect.width < twiceSampleRate and contourRect.height < twiceSampleRate) { // contour too small to sample
            splitTouches.push_back(touches[i]);
            continue;
        }

        // move in small steps until an inside point is found, then increase the step once and repeat again with small steps until inside, etc...
        int step = smallStep;
        for (int y = 0; y < contourRect.height ; y += sampleRate){
            for (int x = 0; x < contourRect.width ; x += sampleRate){
                const Point candidatePoint = Point(contourRect.x + x, contourRect.y + y);

                if (cv::pointPolygonTest(touches[i].contour, candidatePoint, false) < 0) { // sample point outside contour
                    step = smallStep;
                    continue;
                }

                step = bigStep;
                found = true;
                splitTouches.push_back({ candidatePoint, candidatePoint, touches[i].contour, FingerCode::NONE , TouchState::UP, 0});
            }
        }

        if (not found) { // no sample created so use the touch point as it is
            splitTouches.push_back(touches[i]);
            continue;
        }
    }

    return splitTouches;
}

bool TouchDetection::isHovering(const cv::Mat_<float>& depth, cv::Point tipPoint) {
    //Harrison 2016: ".. we examine the 5x5 neighborhood around the tip pixel.
    //                If any pixel is more than 1 cm from the background..."

    Rect roi(tipPoint - Point(2, 2), tipPoint + Point(2, 2));
    Mat depthDiff = m_dmax(roi) - depth(roi);
    if (countNonZero(depthDiff > 10.0) > 0) {
        return true;
    }

    return false;
}

void TouchDetection::initializePixelHistograms(const Mat_<float>& depth) {
    const int width  = depth.cols;
    const int height = depth.rows;

    const int sizes[] = { height, width, m_histogramSize };
    m_pixelDepthHistogram = Mat_<uchar>(3, sizes);

    // Initialize the histograms
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			for (int h = 0; h < m_histogramSize; h++) {
				m_pixelDepthHistogram(y, x, h) = 0;
            }
            if (depth(y, x) != 0.) { // count non-zero depth pixels
                m_pixelDepthHistogram(y, x, m_midHistogram) = 1;
            }
		}
	}

    m_refernceDepth = depth.clone();
}

void TouchDetection::addFrameToPixelHistograms(const Mat_<float>& depth) {
    const int width  = depth.cols;
    const int height = depth.rows;

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			const float depthVal = depth(y, x);
			if (depthVal == 0) { // do not count 0 depth
				continue;
            }

			const float refDepth = m_refernceDepth(y, x);

			// if initial depth is 0 and current depth is not, use the current as the ref depth
			// i.e. the first non-zero depth is used as ref
			if (refDepth == 0) {
				m_refernceDepth(y, x) = depthVal;
				m_pixelDepthHistogram(y, x, m_midHistogram) = 1;
				continue;
			}

			// find the histogram bin from the depth deviation
			const float depthDev = (depthVal - refDepth);
			const int bin = max(0, min(m_histogramSize - 1, cvRound(depthDev) + m_midHistogram));

            m_pixelDepthHistogram(y, x, bin)++;
		}
	}
}

void TouchDetection::findDMaxDMinFromPixelHistogram() {
    const int height = m_refernceDepth.rows;
    const int width  = m_refernceDepth.cols;

    m_dfingermin = cv::Mat_<float>(height, width, 0.0f);
    m_dhandmin   = cv::Mat_<float>(height, width, 0.0f);
    m_dmax       = cv::Mat_<float>(height, width, 0.0f);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const unsigned long refDepth = m_refernceDepth(y, x);

            int maxHist = 0;
            for (int d = 0; d < m_histogramSize; d++) {
                const uchar pixelHistogramBinValue = m_pixelDepthHistogram(y, x, d);

                if (pixelHistogramBinValue >= m_histogramThreshold) {
                    //found histogram bin larger than threshold
                    m_dmax(y, x) = refDepth + (d - m_midHistogram) - m_touchThickness;
                    break;
                }

                //not above threshold - use the histogram bin with maximal value
                if (pixelHistogramBinValue > maxHist) {
                    maxHist = pixelHistogramBinValue;
                    m_dmax(y, x) = refDepth + (d - m_midHistogram) - m_touchThickness;
                }
            }
        }
    }

    //dmin is derived from dmax, by subtarcting finger/hand thickness:
    //finger:
    m_dfingermin = m_dmax - Scalar(m_fingerThickness);
    m_dfingermin.setTo(0, m_dmax == 0); //invalid dmax values nullify dmin
    //hand:
    m_dhandmin = m_dmax - Scalar(m_handThickness);
    m_dhandmin.setTo(0, m_dmax == 0); //invalid dmax values nullify dmin
}
