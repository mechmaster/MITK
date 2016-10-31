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

// forwarddeclaration
class TOFRegistrationViewData;

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

  protected slots:

    /** Starts the registration. When the method is called a seperate UIWorker
      * thread will be run in the background to prevent blocking the GUI.
      */
    void OnStartRegistration();
    void OnRefreshSnapshot();
    
    /** Enables/disables the calculation of the Target Registration Error (TRE).
      */
    void OnEnableTreCalculation();

    /** Enables/disables the trimmed version of the A-ICP algorithm.*/
    void OnEnableTrimming();

  public slots:

    /** Method called when the algorithm is finishes. This method will setup
      * the GUI.
      */
    void OnRegistrationFinished();

  protected:

    virtual void CreateQtPartControl(QWidget *parent) override;

    virtual void SetFocus() override;

    Ui::QmitkTOFRegistrationViewControls m_Controls;

  private:

    TOFRegistrationViewData* d;

    /** Check for the correct input data.*/
    bool CheckInput();

    QString m_TmpPath;
    QString m_SurfacePath;

};

#endif // QmitkTOFRegistrationView_h
