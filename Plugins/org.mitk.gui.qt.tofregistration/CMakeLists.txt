project(org_mitk_gui_qt_tofregistration)

mitk_create_plugin(
  EXPORT_DIRECTIVE TOFREGISTRATION_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  MODULE_DEPENDS MitkToFHardware MitkQtWidgetsExt MitkAlgorithmsExt MitkToFUI
)
