/*===================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center,
Division of Medical and Biological Informatics.
All rights reserved.

This software is distributed WITHOUT ANY WARRANTY; without
even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.

See LICENSE.txt or http://www.mitk.org for details.

===================================================================*/


#ifndef QmitkTOFRegistrationView_h
#define QmitkTOFRegistrationView_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>
#include <QWidget>
#include "ui_QmitkTOFRegistrationViewControls.h"
#include <mitkToFImageGrabber.h>

#include <mitkToFImageGrabber.h>
#include <mitkSurface.h>
#include <mitkToFDistanceImageToSurfaceFilter.h>
#include <mitkToFImageRecorder.h>
#include <mitkToFCompositeFilter.h>
#include <mitkCameraIntrinsics.h>

#include <vtkCamera.h>

// forwarddeclaration
class TOFRegistrationViewData;

class QTimer;

/**
  * \brief Implemenation of a worker thread class.
  *
  *        Worker class that runs the registration
  *        in a seperate QThread to prevent the registration from blocking the
  *        GUI.
  */
class UIWorker : public QObject
{
  Q_OBJECT

  private:
    /** Pimpl with the registration data.*/
    TOFRegistrationViewData* d;

  public slots:

    /** Method that runs the registration algorithm in a seperate QThread.*/
    void RegistrationThreadFunc();

  signals:

    /** Signal emitted when the registration was successful.*/
    void RegistrationFinished();

  public:

    /** Set the data used for the registration.*/
    void SetRegistrationData(TOFRegistrationViewData* data);
};

/**
  \brief QmitkTOFRegistrationView provides a simple UI to register two
         surfaces with the AnisotropicIterativeClosestPointRegistration
         algorithm.

  \sa QmitkAbstractView
  \ingroup ${plugin_target}_internal
*/
class QmitkTOFRegistrationView : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

  public:

    static const std::string VIEW_ID;

    QmitkTOFRegistrationView();

    ~QmitkTOFRegistrationView();

    bool UpdateSurface();

    void SurfaceGenerationInitialize(mitk::ToFDistanceImageToSurfaceFilter::Pointer filter, mitk::ToFImageGrabber::Pointer grabber, mitk::CameraIntrinsics::Pointer intrinsics,
      mitk::DataNode::Pointer surface, vtkSmartPointer<vtkCamera> camera, bool generateSurface = false, bool showAdvancedOptions = true);

  protected slots:

    /** Starts the registration. When the method is called a seperate UIWorker
      * thread will be run in the background to prevent blocking the GUI.
      */
    void OnStartRegistration();
    void OnRefreshSnapshot();
    void OnOldRefreshSnapshot();
    void OnAutoRegistration();
    
    /** Enables/disables the calculation of the Target Registration Error (TRE).
      */
    void OnEnableTreCalculation();

    /** Enables/disables the trimmed version of the A-ICP algorithm.*/
    void OnEnableTrimming();
    void OnUpdate();


    /*!
    \brief Slot triggered from the timer to update the images and visualization
    */
    void OnUpdateCamera();
    /*!
    \brief Slot called when the "Connect" button of the ConnectionWidget is pressed
    */
    void OnToFCameraConnected();
    /*!
    \brief Slot called when the "Disconnect" button of the ConnectionWidget is pressed
    */
    void OnToFCameraDisconnected();
    /*!
    \brief Slot called when the "Start" button of the RecorderWidget is pressed
    */
    void OnToFCameraStarted();
    /*!
    \brief Slot called when the "Stop" button of the RecorderWidget is pressed
    */
    void OnToFCameraStopped();

  public slots:

    /** Method called when the algorithm is finishes. This method will setup
      * the GUI.
      */
    void OnRegistrationFinished();

  protected:

    virtual void CreateQtPartControl(QWidget *parent) override;

    virtual void SetFocus() override;

    void UseToFVisibilitySettings(bool useToF);

    Ui::QmitkTOFRegistrationViewControls m_Controls;


    mitk::Image::Pointer m_MitkDistanceImage; ///< member holding a pointer to the distance image of the selected camera
    mitk::Image::Pointer m_MitkAmplitudeImage; ///< member holding a pointer to the amplitude image of the selected camera
    mitk::Image::Pointer m_MitkIntensityImage; ///< member holding a pointer to the intensity image of the selected camera
    mitk::Surface::Pointer m_Surface; ///< member holding a pointer to the surface generated from the distance image of the selected camera

    mitk::DataNode::Pointer m_DistanceImageNode; ///< DataNode holding the distance image of the selected camera
    mitk::DataNode::Pointer m_AmplitudeImageNode; ///< DataNode holding the amplitude image of the selected camera
    mitk::DataNode::Pointer m_IntensityImageNode; ///< DataNode holding the intensity image of the selected camera
    mitk::DataNode::Pointer m_RGBImageNode; ///< DataNode holding the rgb image of the selected camera
    mitk::DataNode::Pointer m_SurfaceNode; ///< DataNode holding the surface generated from the distanc image of the selected camera

                                           // ToF processing and recording filter
    mitk::ToFImageRecorder::Pointer m_ToFImageRecorder; ///< ToF image recorder used for lossless recording of ToF image data
    mitk::ToFImageGrabber::Pointer m_ToFImageGrabber; ///< Source of a ToF image processing pipeline. Provides pointers to distance, amplitude and intensity image
    mitk::ToFDistanceImageToSurfaceFilter::Pointer m_ToFDistanceImageToSurfaceFilter; ///< Filter for calculating a surface representation from a given distance image
    mitk::ToFCompositeFilter::Pointer m_ToFCompositeFilter; 

    mitk::CameraIntrinsics::Pointer m_CameraIntrinsics; ///< member holding the intrinsic parameters of the camera

  private:

    void ChangeHelperObjectsVisibility(bool visible);

    TOFRegistrationViewData* d;

    /** Check for the correct input data.*/
    bool CheckInput();
    QTimer* m_AutoRegistrationTimer;
    mitk::DataNode::Pointer ReplaceNodeData(std::string nodeName, mitk::BaseData* data);
    bool m_Active;
    vtkSmartPointer<vtkCamera> m_Camera3d;
};

#endif // QmitkTOFRegistrationView_h
