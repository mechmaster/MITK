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


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "QmitkTOFRegistrationView.h"
#include <QmitkStdMultiWidget.h>

// Qt
#include <QMessageBox>
#include <QThread>
#include <QTimer>

// MITK
#include <mitkSurface.h>
#include <mitkPointSet.h>
#include <mitkNodePredicateDataType.h>
#include <mitkAnisotropicIterativeClosestPointRegistration.h>
#include <mitkCovarianceMatrixCalculator.h>
#include <mitkAnisotropicRegistrationCommon.h>
#include <mitkIOUtil.h>
#include <mitkSmartPointerProperty.h>


#include <mitkBaseRenderer.h>
#include <mitkToFDistanceImageToPointSetFilter.h>
#include <mitkTransferFunction.h>
#include <mitkTransferFunctionProperty.h>
#include <mitkToFDeviceFactoryManager.h>
#include <mitkToFCameraDevice.h>
#include <mitkCameraIntrinsicsProperty.h>
#include <mitkSmartPointerProperty.h>
#include <mitkRenderingModeProperty.h>
#include <mitkVtkScalarModeProperty.h>

// vtk
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCleanPolyData.h>
#include <vtkQuadricDecimation.h>

//itk
#include <itksys/SystemTools.hxx>

const std::string QmitkTOFRegistrationView::VIEW_ID = "org.mitk.views.tofregistration";

/**
  * @brief Pimpl holding the datastructures used by the
  *        QmitkTOFRegistrationView.
  */
class TOFRegistrationViewData
{

public:
  QThread* m_RegistrationThread;
  UIWorker* m_Worker;
  double m_Threshold;
  double m_MaxIterations;
  double m_TrimmFactor;
  double m_SearchRadius;

  // anisotropic registration
  mitk::AnisotropicIterativeClosestPointRegistration::Pointer m_AICP;

  // covariance matrix calculator
  mitk::CovarianceMatrixCalculator::Pointer m_MatrixCalculator;

  vtkSmartPointer<vtkCleanPolyData> m_CleanPolyData;

  mitk::Surface::Pointer m_MovingSurface;

  mitk::Surface::Pointer m_FixedSurface;

  vtkPolyData* data_X;

  vtkPolyData* data_Y;

  // c tor
  TOFRegistrationViewData()
    : m_RegistrationThread(new QThread()),
    m_Worker(new UIWorker()),
    m_Threshold(0.5),
    m_MaxIterations(100),
    m_TrimmFactor(0.0),
    m_SearchRadius(30.0),
    m_AICP(mitk::AnisotropicIterativeClosestPointRegistration::New()),
    m_MatrixCalculator(mitk::CovarianceMatrixCalculator::New()),
    m_CleanPolyData(vtkSmartPointer<vtkCleanPolyData>::New()),
    m_MovingSurface(nullptr),
    m_FixedSurface(nullptr)
  {
  }

  // cleanup
  ~TOFRegistrationViewData()
  {
    if (m_RegistrationThread)
      delete m_RegistrationThread;
    if (m_Worker)
      delete m_Worker;

    m_AICP = nullptr;
    m_MatrixCalculator = nullptr;
    m_CleanPolyData = nullptr;
    m_MovingSurface = nullptr;
    m_FixedSurface = nullptr;
  }
};

QmitkTOFRegistrationView::QmitkTOFRegistrationView() :
  m_AutoRegistrationTimer(new QTimer(this)),
  m_MitkDistanceImage(NULL), m_MitkAmplitudeImage(NULL), m_MitkIntensityImage(NULL), m_Surface(NULL)
  , m_DistanceImageNode(NULL), m_AmplitudeImageNode(NULL), m_IntensityImageNode(NULL), m_RGBImageNode(NULL), m_SurfaceNode(NULL)
  , m_ToFImageRecorder(NULL), m_ToFImageGrabber(NULL), m_ToFDistanceImageToSurfaceFilter(NULL), m_ToFCompositeFilter(NULL)
  , m_CameraIntrinsics(NULL)
{
  d = new TOFRegistrationViewData();
}

QmitkTOFRegistrationView::~QmitkTOFRegistrationView()
{
  m_AutoRegistrationTimer->stop();
  m_AutoRegistrationTimer = NULL;

  if (d)
    delete d;
}

void QmitkTOFRegistrationView::SetFocus() {}

void QmitkTOFRegistrationView::CreateQtPartControl(QWidget *parent)
{
  // create GUI widgets from the Qt Designer's .ui file
  m_Controls.setupUi(parent);

  // connect signals and slots
  connect(m_Controls.m_EnableTreCalculation, SIGNAL(clicked()), this, SLOT(OnEnableTreCalculation()));
  connect(m_Controls.m_RegisterSurfaceButton, SIGNAL(clicked()), this, SLOT(OnStartRegistration()));
  connect(m_Controls.m_RefreshSnapshotButton, SIGNAL(clicked()), this, SLOT(OnRefreshSnapshot()));
  connect(m_Controls.m_OldRefreshSnapshotButton, SIGNAL(clicked()), this, SLOT(OnOldRefreshSnapshot()));

  connect(m_Controls.m_AutoRegistrationCheckButton, SIGNAL(clicked()), this, SLOT(OnAutoRegistration()));
  connect(m_Controls.m_EnableTrimming, SIGNAL(clicked()), this, SLOT(OnEnableTrimming()));
  connect(d->m_Worker, SIGNAL(RegistrationFinished()), this, SLOT(OnRegistrationFinished()));
  connect(d->m_RegistrationThread, SIGNAL(started()), d->m_Worker, SLOT(RegistrationThreadFunc()));

  connect(m_AutoRegistrationTimer, SIGNAL(timeout()), this, SLOT(OnUpdate()));

  connect((QObject*)(m_Controls.m_ToFConnectionWidget), SIGNAL(ToFCameraConnected()), this, SLOT(OnToFCameraConnected()));
  connect((QObject*)(m_Controls.m_ToFConnectionWidget), SIGNAL(ToFCameraDisconnected()), this, SLOT(OnToFCameraDisconnected()));
  connect((QObject*)(m_Controls.m_ToFConnectionWidget), SIGNAL(KinectAcquisitionModeChanged()), this, SLOT(OnKinectAcquisitionModeChanged())); // Todo in Widget2
  connect((QObject*)(m_Controls.m_ToFRecorderWidget), SIGNAL(ToFCameraStarted()), this, SLOT(OnToFCameraStarted()));
  connect((QObject*)(m_Controls.m_ToFRecorderWidget), SIGNAL(ToFCameraStopped()), this, SLOT(OnToFCameraStopped()));

  // move the u worker to the thread
  d->m_Worker->moveToThread(d->m_RegistrationThread);

  // setup tooltips
  m_Controls.m_MovingSurfaceComboBox->setToolTip("Set the moving surface of the A-ICP algorithm");
  m_Controls.m_FixedSurfaceComboBox->setToolTip("Set the fixed surface of the A-ICP algorithm");
  m_Controls.m_EnableTreCalculation->setToolTip("Enable the trimmed version of the algorithm.");
  m_Controls.m_TrimmFactorSpinbox->setToolTip("Set the trimmfactor. The algorithm will use a percentage of the Moving pointset for the registration. Valid number are between 0 and 1.");
  m_Controls.m_ThresholdSpinbox->setToolTip("Set the threshold to wich the algorithm will converge.");
  m_Controls.m_MaxIterationsSpinbox->setToolTip("The maximum number of iterations used by the algorithm.");
  m_Controls.m_SearchRadius->setToolTip("Set the search radius in mm for the calculation of the correspondences.");
  m_Controls.m_RegisterSurfaceButton->setToolTip("Start the registration.");
  m_Controls.m_EnableTrimming->setToolTip("Enables the trimmed version of the algorithm.");
  m_Controls.m_TrimmFactorSpinbox->setToolTip("Set teh overlapping part of the surface in %. The valid range is between 0 and 1.");
  m_Controls.m_MovingTargets->setToolTip("Select the targets for the moving surface.");
  m_Controls.m_FixedTargets->setToolTip("Select the targets for the fixed surface.");
  m_Controls.m_RefreshSnapshotButton->setToolTip("Refreshes TOF pointcloud snapshot for registration.");

  // init combo boxes
  m_Controls.m_FixedSurfaceComboBox->SetDataStorage(this->GetDataStorage());
  m_Controls.m_FixedSurfaceComboBox->SetPredicate(mitk::NodePredicateDataType::New("Surface"));

  m_Controls.m_MovingSurfaceComboBox->SetDataStorage(this->GetDataStorage());
  m_Controls.m_MovingSurfaceComboBox->SetPredicate(mitk::NodePredicateDataType::New("Surface"));

  m_Controls.m_MovingTargets->SetDataStorage(this->GetDataStorage());
  m_Controls.m_MovingTargets->SetPredicate(mitk::NodePredicateDataType::New("PointSet"));

  m_Controls.m_FixedTargets->SetDataStorage(this->GetDataStorage());
  m_Controls.m_FixedTargets->SetPredicate(mitk::NodePredicateDataType::New("PointSet"));

  // disable target selection
  m_Controls.m_TargetSelectFrame->setEnabled(false);

  // disable trimming options
  m_Controls.m_TrimmFactorLabel->setEnabled(false);
  m_Controls.m_TrimmFactorSpinbox->setEnabled(false);
}

bool QmitkTOFRegistrationView::CheckInput()
{
  QMessageBox msg;
  msg.setIcon(QMessageBox::Critical);

  if (m_Controls.m_MovingSurfaceComboBox->GetSelectedNode().IsNull() ||
    m_Controls.m_FixedSurfaceComboBox->GetSelectedNode().IsNull())
  {
    const char* message = "No Surfaces selected.";
    MITK_ERROR << message;
    msg.setText(message);
    msg.exec();
    return false;
  }

  if (m_Controls.m_EnableTreCalculation->isChecked())
  {
    if (m_Controls.m_FixedTargets->GetSelectedNode().IsNull() ||
      m_Controls.m_MovingTargets->GetSelectedNode().IsNull())
    {
      const char* message = "TRE calculation is enabled, but no target points are selected.";
      msg.setText(message);
      msg.exec();
      return false;
    }
  }
  return true;
}

void QmitkTOFRegistrationView::OnToFCameraConnected()
{
  MITK_DEBUG << "OnToFCameraConnected";

  this->m_ToFImageGrabber = m_Controls.m_ToFConnectionWidget->GetToFImageGrabber();

  // initialize surface generation
  this->m_ToFDistanceImageToSurfaceFilter = mitk::ToFDistanceImageToSurfaceFilter::New();

  auto generateTriangularMesh = true;
  this->m_ToFDistanceImageToSurfaceFilter->SetGenerateTriangularMesh(generateTriangularMesh);
  this->m_ToFImageGrabber->GetCameraDevice()->SetBoolProperty("GenerateTriangularMesh", generateTriangularMesh);

  // initialize ToFImageRecorder and ToFRecorderWidget
  this->m_ToFImageRecorder = mitk::ToFImageRecorder::New();
  this->m_ToFImageRecorder->SetCameraDevice(this->m_ToFImageGrabber->GetCameraDevice());
  m_Controls.m_ToFRecorderWidget->SetParameter(this->m_ToFImageGrabber, this->m_ToFImageRecorder);
  m_Controls.m_ToFRecorderWidget->setEnabled(true);
  m_Controls.m_ToFRecorderWidget->ResetGUIToInitial();

  // initialize ToFCompositeFilterWidget
  this->m_ToFCompositeFilter = mitk::ToFCompositeFilter::New();

  this->RequestRenderWindowUpdate();
}

void QmitkTOFRegistrationView::OnToFCameraDisconnected()
{
  this->GetDataStorage()->Remove(m_DistanceImageNode);
  if (m_RGBImageNode)
    this->GetDataStorage()->Remove(m_RGBImageNode);
  if (m_AmplitudeImageNode)
    this->GetDataStorage()->Remove(m_AmplitudeImageNode);
  if (m_IntensityImageNode)
    this->GetDataStorage()->Remove(m_IntensityImageNode);
  if (m_SurfaceNode)
    this->GetDataStorage()->Remove(m_SurfaceNode);

  m_Controls.m_ToFRecorderWidget->OnStop();
  m_Controls.m_ToFRecorderWidget->setEnabled(false);
}

void QmitkTOFRegistrationView::OnToFCameraStarted()
{
  if (m_ToFImageGrabber.IsNotNull())
  {
    // initialize camera intrinsics
    if (this->m_ToFImageGrabber->GetProperty("CameraIntrinsics"))
    {
      m_CameraIntrinsics = dynamic_cast<mitk::CameraIntrinsicsProperty*>(this->m_ToFImageGrabber->GetProperty("CameraIntrinsics"))->GetValue();
      MITK_INFO << m_CameraIntrinsics->ToString();
    }
    else
    {
      m_CameraIntrinsics = NULL;
      MITK_ERROR << "No camera intrinsics were found!";
    }

    // set camera intrinsics
    if (m_CameraIntrinsics.IsNotNull())
    {
      this->m_ToFDistanceImageToSurfaceFilter->SetCameraIntrinsics(m_CameraIntrinsics);
    }

    // initial update of image grabber
    this->m_ToFImageGrabber->Update();

    bool hasRGBImage = false;
    m_ToFImageGrabber->GetCameraDevice()->GetBoolProperty("HasRGBImage", hasRGBImage);

    bool hasIntensityImage = false;
    m_ToFImageGrabber->GetCameraDevice()->GetBoolProperty("HasIntensityImage", hasIntensityImage);

    bool hasAmplitudeImage = false;
    m_ToFImageGrabber->GetCameraDevice()->GetBoolProperty("HasAmplitudeImage", hasAmplitudeImage);

    this->m_ToFCompositeFilter->SetInput(0, this->m_ToFImageGrabber->GetOutput(0));
    if (hasAmplitudeImage)
      this->m_ToFCompositeFilter->SetInput(1, this->m_ToFImageGrabber->GetOutput(1));
    if (hasIntensityImage)
      this->m_ToFCompositeFilter->SetInput(2, this->m_ToFImageGrabber->GetOutput(2));

    // initial update of composite filter
    this->m_ToFCompositeFilter->Update();
    this->m_MitkDistanceImage = m_ToFCompositeFilter->GetOutput();
    this->m_DistanceImageNode = ReplaceNodeData("Distance image", m_MitkDistanceImage);

    //std::string rgbFileName;
    //m_ToFImageGrabber->GetCameraDevice()->GetStringProperty("RGBImageFileName", rgbFileName);

    //if (hasRGBImage || (rgbFileName != ""))
    //{
    //  if (m_ToFImageGrabber->GetBoolProperty("IR"))
    //  {
    //    this->m_MitkAmplitudeImage = m_ToFCompositeFilter->GetOutput(1);
    //  }
    //  else
    //  {
    //    this->m_RGBImageNode = ReplaceNodeData("RGB image", this->m_ToFImageGrabber->GetOutput(3));
    //  }
    //}
    //else
    //{
    //  this->m_RGBImageNode = NULL;
    //}

    if (hasAmplitudeImage)
    {
      this->m_MitkAmplitudeImage = m_ToFCompositeFilter->GetOutput(1);
      this->m_AmplitudeImageNode = ReplaceNodeData("Amplitude image", m_MitkAmplitudeImage);
    }

    if (hasIntensityImage)
    {
      this->m_MitkIntensityImage = m_ToFCompositeFilter->GetOutput(2);
      this->m_IntensityImageNode = ReplaceNodeData("Intensity image", m_MitkIntensityImage);
    }

    this->m_ToFDistanceImageToSurfaceFilter->SetInput(0, m_MitkDistanceImage);
    this->m_ToFDistanceImageToSurfaceFilter->SetInput(1, m_MitkAmplitudeImage);
    this->m_ToFDistanceImageToSurfaceFilter->SetInput(2, m_MitkIntensityImage);

    this->m_SurfaceNode = ReplaceNodeData("Surface", NULL);

    SurfaceGenerationInitialize(m_ToFDistanceImageToSurfaceFilter,
      m_ToFImageGrabber,
      m_CameraIntrinsics,
      m_SurfaceNode,
      GetRenderWindowPart()->GetQmitkRenderWindow("3d")->GetRenderer()->GetVtkRenderer()->GetActiveCamera());
  }
}

void QmitkTOFRegistrationView::UseToFVisibilitySettings(bool useToF)
{
  //We need this property for every node.
  mitk::RenderingModeProperty::Pointer renderingModePropertyForTransferFunction = mitk::RenderingModeProperty::New(mitk::RenderingModeProperty::COLORTRANSFERFUNCTION_COLOR);

  // set node properties
  if (m_DistanceImageNode.IsNotNull())
  {
    this->m_DistanceImageNode->SetProperty("visible", mitk::BoolProperty::New(true));
    this->m_DistanceImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("sagittal")->GetRenderWindow()));
    this->m_DistanceImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("coronal")->GetRenderWindow()));
    this->m_DistanceImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("3d")->GetRenderWindow()));
    this->m_DistanceImageNode->SetProperty("Image Rendering.Mode", renderingModePropertyForTransferFunction);
  }
  if (m_AmplitudeImageNode.IsNotNull())
  {
    this->m_AmplitudeImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("axial")->GetRenderWindow()));
    this->m_AmplitudeImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("coronal")->GetRenderWindow()));
    this->m_AmplitudeImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("3d")->GetRenderWindow()));
    this->m_AmplitudeImageNode->SetProperty("Image Rendering.Mode", renderingModePropertyForTransferFunction);
  }
  if (m_IntensityImageNode.IsNotNull())
  {
    this->m_IntensityImageNode->SetProperty("visible", mitk::BoolProperty::New(true));
    this->m_IntensityImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("axial")->GetRenderWindow()));
    this->m_IntensityImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("sagittal")->GetRenderWindow()));
    this->m_IntensityImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("3d")->GetRenderWindow()));
    this->m_IntensityImageNode->SetProperty("Image Rendering.Mode", renderingModePropertyForTransferFunction);
  }
  if ((m_RGBImageNode.IsNotNull()))
  {
    this->m_RGBImageNode->SetProperty("visible", mitk::BoolProperty::New(true));
    this->m_RGBImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("axial")->GetRenderWindow()));
    this->m_RGBImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("sagittal")->GetRenderWindow()));
    this->m_RGBImageNode->SetVisibility(!useToF, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("3d")->GetRenderWindow()));
  }
  // initialize images
  if (m_MitkDistanceImage.IsNotNull())
  {
    mitk::RenderingManager::GetInstance()->InitializeViews(
      this->m_MitkDistanceImage->GetTimeGeometry(), mitk::RenderingManager::REQUEST_UPDATE_2DWINDOWS, true);
  }
  if (this->m_SurfaceNode.IsNotNull())
  {
    QHash<QString, QmitkRenderWindow*> renderWindowHashMap = this->GetRenderWindowPart()->GetQmitkRenderWindows();
    QHashIterator<QString, QmitkRenderWindow*> i(renderWindowHashMap);
    while (i.hasNext()) {
      i.next();
      this->m_SurfaceNode->SetVisibility(false, mitk::BaseRenderer::GetInstance(i.value()->GetRenderWindow()));
    }
    this->m_SurfaceNode->SetVisibility(true, mitk::BaseRenderer::GetInstance(GetRenderWindowPart()->GetQmitkRenderWindow("3d")->GetRenderWindow()));
  }
  //disable/enable gradient background
  this->GetRenderWindowPart()->EnableDecorations(!useToF, QStringList(QString("background")));

  if ((this->m_RGBImageNode.IsNotNull()))
  {
    bool RGBImageHasDifferentResolution = false;
    m_ToFImageGrabber->GetCameraDevice()->GetBoolProperty("RGBImageHasDifferentResolution", RGBImageHasDifferentResolution);
    if (RGBImageHasDifferentResolution)
    {
      //update the display geometry by using the RBG image node. Only for renderwindow coronal
      mitk::RenderingManager::GetInstance()->InitializeView(GetRenderWindowPart()->GetQmitkRenderWindow("coronal")->GetRenderWindow(), this->m_RGBImageNode->GetData()->GetGeometry());
    }
  }
}

void QmitkTOFRegistrationView::OnToFCameraStopped()
{
}

void QmitkTOFRegistrationView::OnUpdateCamera()
{
  if (!UpdateSurface())
  {
    // update pipeline
    this->m_MitkDistanceImage->Update();
  }

  this->RequestRenderWindowUpdate();
}

void QmitkTOFRegistrationView::SurfaceGenerationInitialize(mitk::ToFDistanceImageToSurfaceFilter::Pointer filter,
  mitk::ToFImageGrabber::Pointer grabber,
  mitk::CameraIntrinsics::Pointer intrinsics,
  mitk::DataNode::Pointer surface,
  vtkSmartPointer<vtkCamera> camera,
  bool generateSurface,
  bool showAdvancedOptions)
{
  m_ToFDistanceImageToSurfaceFilter = filter;
  m_ToFImageGrabber = grabber;
  m_CameraIntrinsics = intrinsics;
  m_Active = true;
  m_Camera3d = camera;

  bool hasSurface = false;
  m_ToFImageGrabber->GetCameraDevice()->GetBoolProperty("HasSurface", hasSurface);
  if (hasSurface)
  {
    this->m_Surface = mitk::Surface::New();
  }
  else
  {
    this->m_Surface = this->m_ToFDistanceImageToSurfaceFilter->GetOutput(0);
  }

  m_SurfaceNode = surface;
  m_SurfaceNode->SetData(m_Surface);
}


bool QmitkTOFRegistrationView::UpdateSurface()
{
  bool hasSurface = false;
  this->m_ToFImageGrabber->GetCameraDevice()->GetBoolProperty("HasSurface", hasSurface);

  if (hasSurface)
  {
    auto surfaceProp_ = this->m_ToFImageGrabber->GetCameraDevice()->GetProperty("ToFSurface");
    mitk::SmartPointerProperty::Pointer surfaceProp = dynamic_cast<mitk::SmartPointerProperty *>(surfaceProp_);
    auto surfacePtr = surfaceProp->GetSmartPointer().GetPointer();
    auto vtkPoly = dynamic_cast<mitk::Surface*>(surfacePtr)->GetVtkPolyData();
    this->m_Surface->SetVtkPolyData(vtkPoly);
  }
  else
  {
    this->m_Surface = m_ToFDistanceImageToSurfaceFilter->GetOutput(0);
  }

  //update pipeline
  this->m_Surface->Update();
  this->RequestRenderWindowUpdate();
  return true;
}

void QmitkTOFRegistrationView::OnStartRegistration()
{
  const char* message;
  QMessageBox msg;
  msg.setIcon(QMessageBox::Critical);

  d->m_Threshold = m_Controls.m_ThresholdSpinbox->value();
  d->m_MaxIterations = m_Controls.m_MaxIterationsSpinbox->value();
  d->m_SearchRadius = m_Controls.m_SearchRadius->value();
  d->m_TrimmFactor = 0.0;

  if (m_Controls.m_EnableTrimming->isChecked())
  {
    d->m_TrimmFactor = m_Controls.m_TrimmFactorSpinbox->value();
  }

  if (!CheckInput())
    return;
      
  d->m_MovingSurface = dynamic_cast<mitk::Surface*>(
    m_Controls.m_MovingSurfaceComboBox->GetSelectedNode()->GetData());

  d->m_FixedSurface = dynamic_cast<mitk::Surface*>(
    m_Controls.m_FixedSurfaceComboBox->GetSelectedNode()->GetData());

  // helper
  d->data_X = vtkPolyData::New();

  // helper
  d->data_Y = vtkPolyData::New();

  // clean the poly data to prevent manifold edges and duplicated vertices
  d->m_CleanPolyData->SetInputData(d->m_MovingSurface->GetVtkPolyData());
  d->m_CleanPolyData->Update();
  // copy the polys
  d->data_X->DeepCopy(d->m_CleanPolyData->GetOutput());

  d->m_CleanPolyData->SetInputData(d->m_FixedSurface->GetVtkPolyData());
  d->m_CleanPolyData->Update();
  d->data_Y->DeepCopy(d->m_CleanPolyData->GetOutput());
  
  // sanity check
  if (d->m_FixedSurface.IsNull() || d->m_MovingSurface.IsNull())
  {
    //const char* message = "Input surfaces are NULL.";
    //QMessageBox msg;

    message = "Input surfaces are NULL.";
    msg.setIcon(QMessageBox::Critical);
    msg.setText(message);
    msg.exec();
    MITK_ERROR << message;
    return;
  }

  // enable trimming
  if (m_Controls.m_EnableTrimming->isChecked())
  {
    d->m_TrimmFactor = m_Controls.m_TrimmFactorSpinbox->value();
  }


  // set data into the UI thread
  d->m_Worker->SetRegistrationData(d);

  // start thread
  d->m_RegistrationThread->start();

  // disable registration button
  m_Controls.m_RegisterSurfaceButton->setEnabled(false);


  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

void QmitkTOFRegistrationView::OnRefreshSnapshot()
{
  //Start the Camera
  m_Controls.m_ToFRecorderWidget->OnPlay();

  //Wait to generate image, can't do it any other way
  itksys::SystemTools::Delay(2000);

  //global reinit for updating pipilines on camera images
  mitk::RenderingManager::GetInstance()->InitializeViews();
  this->RequestRenderWindowUpdate();

  //update surface
  bool hasSurface = false;
  mitk::Surface::Pointer surface = mitk::Surface::New();
  OnUpdateCamera();
  UpdateSurface();

  //deleting old surface snapshot node
  mitk::DataNode::Pointer surface_node;
  surface_node = this->GetDataStorage()->GetNamedNode("Surface_snapshot");

  if (surface_node)
  {
    this->GetDataStorage()->Remove(surface_node);
    surface_node = NULL;
  }

  //decimating ToF surface
  vtkPolyData *polydata = m_Surface->GetVtkPolyData();

  vtkQuadricDecimation* decimate = vtkQuadricDecimation::New();
  decimate->SetTargetReduction(0.8);
  decimate->SetInputData(polydata);
  decimate->Update();
  polydata->Delete();
  polydata = decimate->GetOutput();
  polydata->Register(nullptr);

  surface->SetVtkPolyData(polydata);

  //decimate->Delete();

  //creating new surafce snapshot
  surface_node = mitk::DataNode::New();
  surface_node->SetData(surface);
  surface_node->SetName("Surface_snapshot");

  mitk::Color colorProperty;
  colorProperty.SetRed(0);
  colorProperty.SetGreen(1);
  colorProperty.SetBlue(0);

  surface_node->SetColor(colorProperty);

  this->GetDataStorage()->Add(surface_node);

  //set selection

  m_Controls.m_FixedSurfaceComboBox->SetSelectedNode(surface_node);

  QMessageBox msg;
  msg.setIcon(QMessageBox::Critical);

  //is where any nodes with name "Torse"
  mitk::DataNode::Pointer torse_node = this->GetDataStorage()->GetNamedNode("Torse");

  if (!torse_node)
  {
    const char* message = "Load Torse Surface.";
    MITK_ERROR << message;
    msg.setText(message);
    msg.exec();
    return;
  }

  colorProperty.SetRed(1);
  colorProperty.SetGreen(0);
  colorProperty.SetBlue(0);

  torse_node->SetColor(colorProperty);

  m_Controls.m_MovingSurfaceComboBox->SetSelectedNode(torse_node);

  //stop the camera
  m_Controls.m_ToFRecorderWidget->OnStop();

  //surface, distance and amplitude images to unvisible 

  mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode("Distance image");
  if (node) { node->SetVisibility(false); }
  node = this->GetDataStorage()->GetNamedNode("Amplitude image");
  if (node) { node->SetVisibility(false); }
  node = this->GetDataStorage()->GetNamedNode("RGB image");
  if (node) { node->SetVisibility(false); }

  //we need to initialize (reinit) the surface, to make it fit into the renderwindow
  mitk::RenderingManager::GetInstance()->InitializeViews(
    this->m_Surface->GetTimeGeometry(), mitk::RenderingManager::REQUEST_UPDATE_3DWINDOWS, true);

  //global reinit for updating pipilines on camera images
  mitk::RenderingManager::GetInstance()->InitializeViews();
  this->RequestRenderWindowUpdate();

  // correctly place the vtk camera for appropriate surface rendering
  //1m distance to camera should be a nice default value for most cameras
  m_Camera3d->SetPosition(0, 0, 0);
  m_Camera3d->SetViewUp(0, -1, 0);
  m_Camera3d->SetFocalPoint(0, 0, 1);

  if (this->m_CameraIntrinsics.IsNotNull())
  {
    // compute view angle from camera intrinsics
    m_Camera3d->SetViewAngle(mitk::ToFProcessingCommon::CalculateViewAngle(m_CameraIntrinsics, m_ToFImageGrabber->GetCaptureWidth()));
  }
  else
  {
    m_Camera3d->SetViewAngle(45);
  }
  m_Camera3d->SetClippingRange(1, 10000);

  node = this->GetDataStorage()->GetNamedNode("Surface");
  if (node) { node->SetVisibility(false); }

}

void QmitkTOFRegistrationView::OnOldRefreshSnapshot()
{
  //is where any nodes with name "Surface"
  mitk::DataNode::Pointer surface_node = this->GetDataStorage()->GetNamedNode("Surface");

  if (!surface_node)
  {
    return;
  }

  mitk::Surface::Pointer surface = dynamic_cast<mitk::Surface*>(surface_node->GetData());
  mitk::Surface::Pointer clone_full_def_surface = surface->Clone();

  //Deleting old snapshot node
  surface_node = NULL;
  surface_node = this->GetDataStorage()->GetNamedNode("Surface_snapshot");

  if (surface_node)
  {
    this->GetDataStorage()->Remove(surface_node);

    //not working - on second delete fails
    //surface_node->Delete();

    surface_node = NULL;
  }

  //reading new snapshot from drive
  surface = NULL;
  surface = clone_full_def_surface;

  //downmeshing snapshot
  vtkPolyData *polydata = surface->GetVtkPolyData();
  vtkQuadricDecimation* decimate = vtkQuadricDecimation::New();
  decimate->SetTargetReduction(0.8);
  decimate->SetInputData(polydata);
  decimate->Update();
  polydata->Delete();
  polydata = decimate->GetOutput();
  polydata->Register(nullptr);

  surface->SetVtkPolyData(polydata);

  //decimate->Delete();
  surface_node = mitk::DataNode::New();
  surface_node->SetData(surface);
  surface_node->SetName("Surface_snapshot");

  mitk::Color colorProperty;
  colorProperty.SetRed(0);
  colorProperty.SetGreen(1);
  colorProperty.SetBlue(0);

  surface_node->SetColor(colorProperty);

  this->GetDataStorage()->Add(surface_node);

  //set selection

  m_Controls.m_FixedSurfaceComboBox->SetSelectedNode(surface_node);

  //is where any nodes with name "Torse"
  mitk::DataNode::Pointer torse_node = this->GetDataStorage()->GetNamedNode("Torse");

  if (!torse_node)
  {
    return;
  }

  colorProperty.SetRed(1);
  colorProperty.SetGreen(0);
  colorProperty.SetBlue(0);

  torse_node->SetColor(colorProperty);

  m_Controls.m_MovingSurfaceComboBox->SetSelectedNode(torse_node);

  //OnStartRegistration();
}

void QmitkTOFRegistrationView::OnUpdate()
{
  ChangeHelperObjectsVisibility(true);
  OnRefreshSnapshot();
  OnStartRegistration();
  ChangeHelperObjectsVisibility(false);
}

void QmitkTOFRegistrationView::ChangeHelperObjectsVisibility(bool visible)
{
  mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode("Distance image");
  if (node) { node->SetVisibility(visible); }
  node = this->GetDataStorage()->GetNamedNode("Amplitude image");
  if (node) { node->SetVisibility(visible); }
  node = this->GetDataStorage()->GetNamedNode("RGB image");
  if (node) { node->SetVisibility(visible); }
  node = this->GetDataStorage()->GetNamedNode("Surface");
  if (node) { node->SetVisibility(visible); }
  node = this->GetDataStorage()->GetNamedNode("Surface_snapshot");
  if (node) { node->SetVisibility(visible); }
}

void QmitkTOFRegistrationView::OnAutoRegistration()
{
  if (m_Controls.m_AutoRegistrationCheckButton->isChecked())
  {
    int intervalInSeconds = m_Controls.m_AutoRegIntervalSecSpinBox->value();

    if (intervalInSeconds < 10)
    {
      //show message
      QMessageBox msg;
      msg.setIcon(QMessageBox::Critical);
      const char* message = "Auto Registration is enabled, but interval is lower than 10 seconds.";
      msg.setText(message);
      msg.exec();
      return;

      if (m_AutoRegistrationTimer->isActive())
      {
        m_AutoRegistrationTimer->stop();
      }
    }

    if (!m_AutoRegistrationTimer->isActive())
    {
      m_AutoRegistrationTimer->start(intervalInSeconds * 1000);
    }
  }
  else
  {
    if (m_AutoRegistrationTimer->isActive())
    {
      m_AutoRegistrationTimer->stop();
    }
  }
}

void QmitkTOFRegistrationView::OnEnableTreCalculation()
{
  if (m_Controls.m_EnableTreCalculation->isChecked())
    m_Controls.m_TargetSelectFrame->setEnabled(true);
  else
    m_Controls.m_TargetSelectFrame->setEnabled(false);
}

void QmitkTOFRegistrationView::OnEnableTrimming()
{
  if (m_Controls.m_EnableTrimming->isChecked())
  {
    // disable trimming options
    m_Controls.m_TrimmFactorLabel->setEnabled(true);
    m_Controls.m_TrimmFactorSpinbox->setEnabled(true);
  }
  else {
    // disable trimming options
    m_Controls.m_TrimmFactorLabel->setEnabled(false);
    m_Controls.m_TrimmFactorSpinbox->setEnabled(false);
  }
}

void QmitkTOFRegistrationView::OnRegistrationFinished()
{
  typedef itk::Matrix<double, 3, 3> Matrix3x3;
  typedef itk::Vector<double, 3> TranslationVector;

  double tre = -1.0;
  Matrix3x3 rotation = d->m_AICP->GetRotation();
  TranslationVector translation = d->m_AICP->GetTranslation();

  // exit the thread
  d->m_RegistrationThread->quit();

  MITK_INFO << "Rotation: \n" << rotation << "Translation: " << translation;
  MITK_INFO << "FRE: " << d->m_AICP->GetFRE();

  // compute TRE
  if (m_Controls.m_EnableTreCalculation->isChecked())
  {
    mitk::PointSet* movingTargets = dynamic_cast<mitk::PointSet*> (
      m_Controls.m_MovingTargets->GetSelectedNode()->GetData());

    mitk::PointSet* fixedTargets = dynamic_cast<mitk::PointSet*> (
      m_Controls.m_FixedTargets->GetSelectedNode()->GetData());

    // sanity check
    if (movingTargets && fixedTargets)
    {
      // swap the moving and the fixed point set, since we use the inverse
      // transform
      tre = mitk::AnisotropicRegistrationCommon::ComputeTargetRegistrationError(
        movingTargets,
        fixedTargets,
        rotation,
        translation
      );
      MITK_INFO << "TRE: " << tre;

      // transform the fixed point set
      for (int i = 0; i < movingTargets->GetSize(); ++i)
      {
        mitk::Point3D p = movingTargets->GetPoint(i);

        p = rotation * p + translation;

        movingTargets->SetPoint(i, p);
      }
    }
  }
  // display result in textbox ( the inverse transform )
  QString text("");
  std::ostringstream oss;

  oss << "<b>Iterations:</b> " << d->m_AICP->GetNumberOfIterations()
    << "<br><b>FRE:</b> " << d->m_AICP->GetFRE()
    << "<br><b>TRE:</b> ";

  if (tre != -1.0)
    oss << tre;
  else
    oss << "N/A";

  oss << "<br><br><b>Rotation:</b><br>";

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      oss << rotation[i][j] << " ";
    oss << "<br>";
  }

  oss << "<br><b>Translation:</b><br>" << translation << "<br>";

  std::string s(oss.str());
  text.append(s.c_str());

  m_Controls.m_TextEdit->clear();
  m_Controls.m_TextEdit->append(text);

  mitk::AnisotropicRegistrationCommon::TransformPoints(
    d->m_MovingSurface->GetVtkPolyData()->GetPoints(),
    d->m_MovingSurface->GetVtkPolyData()->GetPoints(),
    rotation,
    translation
  );

  // set modified flag to update rendering
  d->m_MovingSurface->GetVtkPolyData()->Modified();

  // reanable registration button
  m_Controls.m_RegisterSurfaceButton->setEnabled(true);

  //update view
  mitk::RenderingManager::GetInstance()->RequestUpdateAll();
}

mitk::DataNode::Pointer QmitkTOFRegistrationView::ReplaceNodeData(std::string nodeName, mitk::BaseData* data)
{
  mitk::DataNode::Pointer node = this->GetDataStorage()->GetNamedNode(nodeName);
  if (node.IsNull())
  {
    node = mitk::DataNode::New();
    node->SetName(nodeName);
    node->SetBoolProperty("binary", false);
    node->SetData(data);
    this->GetDataStorage()->Add(node);
  }
  else
  {
    node->SetData(data);
  }
  return node;
}

void UIWorker::SetRegistrationData(TOFRegistrationViewData *data)
{
  this->d = data;
}

void UIWorker::RegistrationThreadFunc()
{
  typedef itk::Matrix<double, 3, 3> Matrix3x3;
  typedef itk::Vector<double, 3> TranslationVector;
  typedef std::vector<Matrix3x3> CovarianceMatrixList;

  // moving surface
  mitk::Surface::Pointer X = mitk::Surface::New();

  // fixed surface
  mitk::Surface::Pointer Y = mitk::Surface::New();

  //// copy the polys
  X->SetVtkPolyData(d->data_X);
  Y->SetVtkPolyData(d->data_Y);

  // compute the covariance matrices for the moving surface (X)
  d->m_MatrixCalculator->SetInputSurface(X);
  d->m_MatrixCalculator->ComputeCovarianceMatrices();
  CovarianceMatrixList sigmas_X = d->m_MatrixCalculator->GetCovarianceMatrices();
  const double meanVarX = d->m_MatrixCalculator->GetMeanVariance();

  // compute the covariance matrices for the fixed surface (Y)
  d->m_MatrixCalculator->SetInputSurface(Y);
  d->m_MatrixCalculator->ComputeCovarianceMatrices();
  CovarianceMatrixList sigmas_Y = d->m_MatrixCalculator->GetCovarianceMatrices();
  const double meanVarY = d->m_MatrixCalculator->GetMeanVariance();

  // the FRE normalization factor
  const double normalizationFactor = sqrt(meanVarX + meanVarY);

  // set up parameters
  d->m_AICP->SetMovingSurface(X);
  d->m_AICP->SetFixedSurface(Y);
  d->m_AICP->SetCovarianceMatricesMovingSurface(sigmas_X);
  d->m_AICP->SetCovarianceMatricesFixedSurface(sigmas_Y);
  d->m_AICP->SetFRENormalizationFactor(normalizationFactor);
  d->m_AICP->SetMaxIterations(d->m_MaxIterations);
  d->m_AICP->SetSearchRadius(d->m_SearchRadius);
  d->m_AICP->SetThreshold(d->m_Threshold);
  d->m_AICP->SetTrimmFactor(d->m_TrimmFactor);

  // run the algorithm
  d->m_AICP->Update();

  d->data_X->Delete();
  d->data_Y->Delete();

  emit RegistrationFinished();
}



