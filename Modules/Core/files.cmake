file(GLOB_RECURSE H_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/include/*")

set(CPP_FILES
  mitkCoreActivator.cpp
  mitkCoreObjectFactoryBase.cpp
  mitkCoreObjectFactory.cpp
  mitkCoreServices.cpp
  mitkException.cpp

  Algorithms/mitkBaseDataSource.cpp
  Algorithms/mitkClippedSurfaceBoundsCalculator.cpp
  Algorithms/mitkCompareImageDataFilter.cpp
  Algorithms/mitkCompositePixelValueToString.cpp
  Algorithms/mitkConvert2Dto3DImageFilter.cpp
  Algorithms/mitkDataNodeSource.cpp
  Algorithms/mitkExtractSliceFilter.cpp
  Algorithms/mitkHistogramGenerator.cpp
  Algorithms/mitkImageChannelSelector.cpp
  Algorithms/mitkImageSliceSelector.cpp
  Algorithms/mitkImageSource.cpp
  Algorithms/mitkImageTimeSelector.cpp
  Algorithms/mitkImageToImageFilter.cpp
  Algorithms/mitkImageToSurfaceFilter.cpp
  Algorithms/mitkMultiComponentImageDataComparisonFilter.cpp
  Algorithms/mitkPlaneGeometryDataToSurfaceFilter.cpp
  Algorithms/mitkPointSetSource.cpp
  Algorithms/mitkPointSetToPointSetFilter.cpp
  Algorithms/mitkRGBToRGBACastImageFilter.cpp
  Algorithms/mitkSubImageSelector.cpp
  Algorithms/mitkSurfaceSource.cpp
  Algorithms/mitkSurfaceToImageFilter.cpp
  Algorithms/mitkSurfaceToSurfaceFilter.cpp
  Algorithms/mitkUIDGenerator.cpp
  Algorithms/mitkVolumeCalculator.cpp

  Controllers/mitkBaseController.cpp
  Controllers/mitkCallbackFromGUIThread.cpp
  Controllers/mitkCameraController.cpp
  Controllers/mitkCameraRotationController.cpp
  Controllers/mitkLimitedLinearUndo.cpp
  Controllers/mitkOperationEvent.cpp
  Controllers/mitkPlanePositionManager.cpp
  Controllers/mitkProgressBar.cpp
  Controllers/mitkRenderingManager.cpp
  Controllers/mitkSliceNavigationController.cpp
  Controllers/mitkSlicesCoordinator.cpp
  Controllers/mitkStatusBar.cpp
  Controllers/mitkStepper.cpp
  Controllers/mitkTestManager.cpp
  Controllers/mitkUndoController.cpp
  Controllers/mitkVerboseLimitedLinearUndo.cpp
  Controllers/mitkVtkLayerController.cpp
  DataManagement/mitkArbitraryTimeGeometry.cpp
  DataManagement/mitkAbstractTransformGeometry.cpp
  DataManagement/mitkAnnotationProperty.cpp
  DataManagement/mitkApplicationCursor.cpp
  DataManagement/mitkApplyTransformMatrixOperation.cpp
  DataManagement/mitkBaseData.cpp
  DataManagement/mitkBaseGeometry.cpp
  DataManagement/mitkBaseProperty.cpp
  DataManagement/mitkChannelDescriptor.cpp
  DataManagement/mitkClippingProperty.cpp
  DataManagement/mitkColorProperty.cpp
  DataManagement/mitkDataNode.cpp
  DataManagement/mitkDataStorage.cpp
  DataManagement/mitkEnumerationProperty.cpp
  DataManagement/mitkFloatPropertyExtension.cpp
  DataManagement/mitkGeometry3D.cpp
  DataManagement/mitkGeometryData.cpp
  DataManagement/mitkGeometryTransformHolder.cpp
  DataManagement/mitkGroupTagProperty.cpp
  DataManagement/mitkImageAccessorBase.cpp
  DataManagement/mitkImageCaster.cpp
  DataManagement/mitkImageCastPart1.cpp
  DataManagement/mitkImageCastPart2.cpp
  DataManagement/mitkImageCastPart3.cpp
  DataManagement/mitkImageCastPart4.cpp
  DataManagement/mitkImage.cpp
  DataManagement/mitkImageDataItem.cpp
  DataManagement/mitkImageDescriptor.cpp
  DataManagement/mitkImageReadAccessor.cpp
  DataManagement/mitkImageStatisticsHolder.cpp
  DataManagement/mitkImageVtkAccessor.cpp
  DataManagement/mitkImageVtkReadAccessor.cpp
  DataManagement/mitkImageVtkWriteAccessor.cpp
  DataManagement/mitkImageWriteAccessor.cpp
  DataManagement/mitkIntPropertyExtension.cpp
  DataManagement/mitkIPersistenceService.cpp
  DataManagement/mitkIPropertyAliases.cpp
  DataManagement/mitkIPropertyDescriptions.cpp
  DataManagement/mitkIPropertyExtensions.cpp
  DataManagement/mitkIPropertyFilters.cpp
  DataManagement/mitkIPropertyPersistence.cpp
  DataManagement/mitkLandmarkProjectorBasedCurvedGeometry.cpp
  DataManagement/mitkLandmarkProjector.cpp
  DataManagement/mitkLevelWindow.cpp
  DataManagement/mitkLevelWindowManager.cpp
  DataManagement/mitkLevelWindowPreset.cpp
  DataManagement/mitkLevelWindowProperty.cpp
  DataManagement/mitkLine.cpp
  DataManagement/mitkLookupTable.cpp
  DataManagement/mitkLookupTableProperty.cpp
  DataManagement/mitkLookupTables.cpp # specializations of GenericLookupTable
  DataManagement/mitkMaterial.cpp
  DataManagement/mitkMemoryUtilities.cpp
  DataManagement/mitkModalityProperty.cpp
  DataManagement/mitkModifiedLock.cpp
  DataManagement/mitkNodePredicateAnd.cpp
  DataManagement/mitkNodePredicateBase.cpp
  DataManagement/mitkNodePredicateCompositeBase.cpp
  DataManagement/mitkNodePredicateData.cpp
  DataManagement/mitkNodePredicateDataType.cpp
  DataManagement/mitkNodePredicateDimension.cpp
  DataManagement/mitkNodePredicateFirstLevel.cpp
  DataManagement/mitkNodePredicateNot.cpp
  DataManagement/mitkNodePredicateOr.cpp
  DataManagement/mitkNodePredicateProperty.cpp
  DataManagement/mitkNodePredicateSource.cpp
  DataManagement/mitkNumericConstants.cpp
  DataManagement/mitkPlaneGeometry.cpp
  DataManagement/mitkPlaneGeometryData.cpp
  DataManagement/mitkPlaneOperation.cpp
  DataManagement/mitkPlaneOrientationProperty.cpp
  DataManagement/mitkPointOperation.cpp
  DataManagement/mitkPointSet.cpp
  DataManagement/mitkPointSetShapeProperty.cpp
  DataManagement/mitkProperties.cpp
  DataManagement/mitkPropertyAliases.cpp
  DataManagement/mitkPropertyDescriptions.cpp
  DataManagement/mitkPropertyExtension.cpp
  DataManagement/mitkPropertyExtensions.cpp
  DataManagement/mitkPropertyFilter.cpp
  DataManagement/mitkPropertyFilters.cpp
  DataManagement/mitkPropertyList.cpp
  DataManagement/mitkPropertyListReplacedObserver.cpp
  DataManagement/mitkPropertyObserver.cpp
  DataManagement/mitkPropertyPersistence.cpp
  DataManagement/mitkPropertyPersistenceInfo.cpp
  DataManagement/mitkProportionalTimeGeometry.cpp
  DataManagement/mitkRenderingModeProperty.cpp
  DataManagement/mitkResliceMethodProperty.cpp
  DataManagement/mitkRestorePlanePositionOperation.cpp
  DataManagement/mitkRotationOperation.cpp
  DataManagement/mitkScaleOperation.cpp
  DataManagement/mitkShaderProperty.cpp
  DataManagement/mitkSlicedData.cpp
  DataManagement/mitkSlicedGeometry3D.cpp
  DataManagement/mitkSmartPointerProperty.cpp
  DataManagement/mitkStandaloneDataStorage.cpp
  DataManagement/mitkStringProperty.cpp
  DataManagement/mitkSurface.cpp
  DataManagement/mitkSurfaceOperation.cpp
  DataManagement/mitkThinPlateSplineCurvedGeometry.cpp
  DataManagement/mitkTimeGeometry.cpp
  DataManagement/mitkTransferFunction.cpp
  DataManagement/mitkTransferFunctionInitializer.cpp
  DataManagement/mitkTransferFunctionProperty.cpp
  DataManagement/mitkTemporoSpatialStringProperty.cpp
  DataManagement/mitkVector.cpp
  DataManagement/mitkVectorProperty.cpp
  DataManagement/mitkVtkInterpolationProperty.cpp
  DataManagement/mitkVtkRepresentationProperty.cpp
  DataManagement/mitkVtkResliceInterpolationProperty.cpp
  DataManagement/mitkVtkScalarModeProperty.cpp
  DataManagement/mitkVtkVolumeRenderingProperty.cpp
  DataManagement/mitkWeakPointerProperty.cpp

  Interactions/mitkAction.cpp
  Interactions/mitkBindDispatcherInteractor.cpp
  Interactions/mitkCrosshairPositionEvent.cpp
  Interactions/mitkDataInteractor.cpp
  Interactions/mitkDispatcher.cpp
  Interactions/mitkDisplayCoordinateOperation.cpp
  Interactions/mitkDisplayInteractor.cpp
  Interactions/mitkEventConfig.cpp
  Interactions/mitkEventFactory.cpp
  Interactions/mitkEventRecorder.cpp
  Interactions/mitkEventStateMachine.cpp
  Interactions/mitkInteractionEventConst.cpp
  Interactions/mitkInteractionEvent.cpp
  Interactions/mitkInteractionEventHandler.cpp
  Interactions/mitkInteractionEventObserver.cpp
  Interactions/mitkInteractionKeyEvent.cpp
  Interactions/mitkInteractionPositionEvent.cpp
  Interactions/mitkInternalEvent.cpp
  Interactions/mitkMouseDoubleClickEvent.cpp
  Interactions/mitkMouseModeSwitcher.cpp
  Interactions/mitkMouseMoveEvent.cpp
  Interactions/mitkMousePressEvent.cpp
  Interactions/mitkMouseReleaseEvent.cpp
  Interactions/mitkMouseWheelEvent.cpp
  Interactions/mitkPointSetDataInteractor.cpp
  Interactions/mitkSinglePointDataInteractor.cpp
  Interactions/mitkStateMachineAction.cpp
  Interactions/mitkStateMachineCondition.cpp
  Interactions/mitkStateMachineContainer.cpp
  Interactions/mitkStateMachineState.cpp
  Interactions/mitkStateMachineTransition.cpp
  Interactions/mitkVtkEventAdapter.cpp
  Interactions/mitkVtkInteractorStyle.cxx
  Interactions/mitkXML2EventParser.cpp
  Interactions/mitkPickingEventObserver.cpp
  Interactions/mitkDataNodePickingEventObserver.cpp

  IO/mitkAbstractFileIO.cpp
  IO/mitkAbstractFileReader.cpp
  IO/mitkAbstractFileWriter.cpp
  IO/mitkCustomMimeType.cpp
  IO/mitkDicomSeriesReader.cpp
  IO/mitkDicomSeriesReaderService.cpp
  IO/mitkDicomSR_GantryTiltInformation.cpp
  IO/mitkDicomSR_ImageBlockDescriptor.cpp
  IO/mitkDicomSR_LoadDICOMRGBPixel4D.cpp
  IO/mitkDicomSR_LoadDICOMRGBPixel.cpp
  IO/mitkDicomSR_LoadDICOMScalar4D.cpp
  IO/mitkDicomSR_LoadDICOMScalar.cpp
  IO/mitkDicomSR_SliceGroupingResult.cpp
  IO/mitkFileReader.cpp
  IO/mitkFileReaderRegistry.cpp
  IO/mitkFileReaderSelector.cpp
  IO/mitkFileReaderWriterBase.cpp
  IO/mitkFileWriter.cpp
  IO/mitkFileWriterRegistry.cpp
  IO/mitkFileWriterSelector.cpp
  IO/mitkGeometry3DToXML.cpp
  IO/mitkIFileIO.cpp
  IO/mitkIFileReader.cpp
  IO/mitkIFileWriter.cpp
  IO/mitkGeometryDataReaderService.cpp
  IO/mitkGeometryDataWriterService.cpp
  IO/mitkImageGenerator.cpp
  IO/mitkImageVtkLegacyIO.cpp
  IO/mitkImageVtkXmlIO.cpp
  IO/mitkIMimeTypeProvider.cpp
  IO/mitkIOConstants.cpp
  IO/mitkIOMimeTypes.cpp
  IO/mitkIOUtil.cpp
  IO/mitkItkImageIO.cpp
  IO/mitkItkLoggingAdapter.cpp
  IO/mitkLegacyFileReaderService.cpp
  IO/mitkLegacyFileWriterService.cpp
  IO/mitkLocaleSwitch.cpp
  IO/mitkLog.cpp
  IO/mitkMimeType.cpp
  IO/mitkMimeTypeProvider.cpp
  IO/mitkOperation.cpp
  IO/mitkPixelType.cpp
  IO/mitkPointSetReaderService.cpp
  IO/mitkPointSetWriterService.cpp
  IO/mitkProportionalTimeGeometryToXML.cpp
  IO/mitkRawImageFileReader.cpp
  IO/mitkStandardFileLocations.cpp
  IO/mitkSurfaceStlIO.cpp
  IO/mitkSurfaceVtkIO.cpp
  IO/mitkSurfaceVtkLegacyIO.cpp
  IO/mitkSurfaceVtkXmlIO.cpp
  IO/mitkVtkLoggingAdapter.cpp

  Rendering/mitkAbstractOverlayLayouter.cpp
  Rendering/mitkBaseRenderer.cpp
  #Rendering/mitkGLMapper.cpp Moved to deprecated LegacyGL Module
  Rendering/mitkGradientBackground.cpp
  Rendering/mitkImageVtkMapper2D.cpp
  Rendering/mitkIShaderRepository.cpp
  Rendering/mitkManufacturerLogo.cpp
  Rendering/mitkMapper.cpp
  Rendering/mitkOverlay.cpp
  Rendering/mitkOverlayManager.cpp
  Rendering/mitkPlaneGeometryDataMapper2D.cpp
  Rendering/mitkPlaneGeometryDataVtkMapper3D.cpp
  Rendering/mitkPointSetVtkMapper2D.cpp
  Rendering/mitkPointSetVtkMapper3D.cpp
  Rendering/mitkRenderWindowBase.cpp
  Rendering/mitkRenderWindow.cpp
  Rendering/mitkRenderWindowFrame.cpp
  #Rendering/mitkSurfaceGLMapper2D.cpp Moved to deprecated LegacyGL Module
  Rendering/mitkSurfaceVtkMapper2D.cpp
  Rendering/mitkSurfaceVtkMapper3D.cpp
  Rendering/mitkVtkEventProvider.cpp
  Rendering/mitkVtkMapper.cpp
  Rendering/mitkVtkOverlay2D.cpp
  Rendering/mitkVtkOverlay3D.cpp
  Rendering/mitkVtkOverlay.cpp
  Rendering/mitkVtkPropRenderer.cpp
  Rendering/mitkVtkWidgetRendering.cpp
  Rendering/vtkMitkLevelWindowFilter.cpp
  Rendering/vtkMitkRectangleProp.cpp
  Rendering/vtkMitkRenderProp.cpp
  Rendering/vtkMitkThickSlicesFilter.cpp
  Rendering/vtkNeverTranslucentTexture.cpp
)

set(RESOURCE_FILES
Interactions/globalConfig.xml
Interactions/DisplayInteraction.xml
Interactions/DisplayConfig.xml
Interactions/DisplayConfigPACS.xml
Interactions/DisplayConfigPACSPan.xml
Interactions/DisplayConfigPACSScroll.xml
Interactions/DisplayConfigPACSZoom.xml
Interactions/DisplayConfigPACSLevelWindow.xml
Interactions/DisplayConfigMITK.xml
Interactions/DisplayConfigMITKNoCrosshair.xml
Interactions/DisplayConfigMITKRotation.xml
Interactions/DisplayConfigMITKRotationUnCoupled.xml
Interactions/DisplayConfigMITKSwivel.xml
Interactions/DisplayConfigMITKLimited.xml
Interactions/PointSet.xml
Interactions/Legacy/StateMachine.xml
Interactions/Legacy/DisplayConfigMITKTools.xml
Interactions/PointSetConfig.xml

mitkLevelWindowPresets.xml
)
