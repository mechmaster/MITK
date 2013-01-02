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

#ifndef MITKEVENTINTERACTOR_H_
#define MITKEVENTINTERACTOR_H_

#include "itkObject.h"
#include "itkSmartPointer.h"
#include "itkObjectFactory.h"
#include "mitkCommon.h"
#include <MitkExports.h>
#include "mitkEventStateMachine.h"

namespace mitk
{
  /** Base class from with data interactors are to be derived.
   * Provides an interface that is relevant for the interactor to work together with the dispatcher.
   * To implement a new interactor overwrite the ConnectActionsAndFunctions to connect the actions.
   */

  class DataNode;
  class MITK_CORE_EXPORT DataInteractor: public EventStateMachine
  {

  public:
    typedef itk::SmartPointer<DataNode> NodeType;
    mitkClassMacro(DataInteractor, EventStateMachine);
    itkNewMacro(Self);

    /**
     * Overload operator to compare Interactors. Criteria is the layer of the associated DataNode.
     * Note: This is intentionally implemented using 'greater than' to achieve a list which is sorted in descending order
     * (DataNodes on top are first in list)
     */
    bool operator<(const DataInteractor::Pointer dataInteractor);

    void SetDataNode(NodeType);
    /**
     * Sets the maximum distance that is accepted when looking for a point at a certain position using the GetPointIndexByPosition function.
     */
    void SetAccuracy(float accuracy);

    NodeType GetDataNode();
    int GetLayer();

  protected:
    DataInteractor();
    virtual ~DataInteractor();
    virtual void ConnectActionsAndFunctions();

    /** Function that iterates over all points in datanode to check if it contains a point near the pointer position.
     * If a point is found its index-position is returned, else -1 is returned.
     */
    virtual int GetPointIndexByPosition(Point3D position);

  private:
    NodeType m_DataNode;
    float m_SelectionAccuracy;
  };

} /* namespace mitk */
#endif /* MITKEVENTINTERACTOR_H_ */
