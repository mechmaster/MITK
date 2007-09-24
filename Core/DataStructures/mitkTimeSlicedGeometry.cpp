/*=========================================================================

Program:   Medical Imaging & Interaction Toolkit
Module:    $RCSfile$
Language:  C++
Date:      $Date$
Version:   $Revision$

Copyright (c) German Cancer Research Center, Division of Medical and
Biological Informatics. All rights reserved.
See MITKCopyright.txt or http://www.mitk.org/copyright.html for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notices for more information.

=========================================================================*/


#include "mitkTimeSlicedGeometry.h"
#include <mitkXMLWriter.h>
#include <mitkXMLReader.h>

const std::string mitk::TimeSlicedGeometry::EVENLY_TIMED = "EVENLY_TIMED";
const std::string mitk::TimeSlicedGeometry::TIME_STEPS = "TIME_STEPS";

void mitk::TimeSlicedGeometry::UpdateInformation()
{
  if(m_TimeSteps==0) return;

  unsigned long maxModifiedTime = 0, curModifiedTime;

  mitk::ScalarType stmin, stmax;
  stmin= ScalarTypeNumericTraits::NonpositiveMin();
  stmax= ScalarTypeNumericTraits::max();

  TimeBounds timeBounds;
  timeBounds[0]=stmax; timeBounds[1]=stmin;

  mitk::BoundingBox::Pointer boundingBox=mitk::BoundingBox::New();
 
  mitk::BoundingBox::PointsContainer::Pointer pointscontainer=mitk::BoundingBox::PointsContainer::New();
  mitk::ScalarType nullpoint[]={0,0,0};
  mitk::BoundingBox::PointType p(nullpoint);

  unsigned int t;

  mitk::Geometry3D* geometry3d;
  mitk::BoundingBox::ConstPointer nextBoundingBox;
  mitk::BoundingBox::PointIdentifier pointid=0;

  for(t=0; t < m_TimeSteps; ++t)
  {
    geometry3d = GetGeometry3D(t);
    assert(geometry3d!=NULL);

    curModifiedTime = geometry3d->GetMTime();
    if(maxModifiedTime < curModifiedTime)
      maxModifiedTime = curModifiedTime;

    const TimeBounds & curTimeBounds = geometry3d->GetTimeBounds();
    if((curTimeBounds[0] > stmin) && (curTimeBounds[0] < timeBounds[0]))
      timeBounds[0] = curTimeBounds[0];
    if((curTimeBounds[1] < stmax) && (curTimeBounds[1] > timeBounds[1]))
      timeBounds[1] = curTimeBounds[1];

    nextBoundingBox = geometry3d->GetBoundingBox();
    assert(nextBoundingBox.IsNotNull());
    const mitk::BoundingBox::PointsContainer * nextPoints = nextBoundingBox->GetPoints();
    if(nextPoints!=NULL)
    {
      mitk::BoundingBox::PointsContainer::ConstIterator pointsIt = nextPoints->Begin();

      while (pointsIt != nextPoints->End() )
      {
        pointscontainer->InsertElement( pointid++, pointsIt->Value());
        ++pointsIt;
      }
    }
  }

  if(!(timeBounds[0] < stmax))
  {
    timeBounds[0] = stmin;
    timeBounds[1] = stmax;
  }

  m_TimeBounds = timeBounds;
  assert(timeBounds[0]<=timeBounds[1]);

  boundingBox->SetPoints(pointscontainer);

  boundingBox->ComputeBoundingBox();

  m_BoundingBox = boundingBox;

  SetIndexToWorldTransform(GetGeometry3D(0)->GetIndexToWorldTransform());

  if(this->GetMTime() < maxModifiedTime)
    Modified();
}

mitk::Geometry3D* mitk::TimeSlicedGeometry::GetGeometry3D(int t) const
{
  mitk::Geometry3D::Pointer geometry3d = NULL;
  if(IsValidTime(t))
  {
    geometry3d = m_Geometry3Ds[t];
    //if (a) we don't have a Geometry3D stored for the requested time, 
    //(b) m_EvenlyTimed is activated and (c) the first geometry (t=0) 
    //is set, then we clone the geometry and set the m_TimeBounds accordingly.
    if((m_EvenlyTimed) && (geometry3d.IsNull()))
    {
      const Geometry3D* firstgeometry=m_Geometry3Ds[0].GetPointer();

      assert(firstgeometry != NULL);

      mitk::Geometry3D::Pointer requestedgeometry;
      requestedgeometry = dynamic_cast<Geometry3D*>(firstgeometry->Clone().GetPointer());
      if ( requestedgeometry.IsNull() ) itkExceptionMacro("Geometry is NULL!");

      TimeBounds timebounds = requestedgeometry->GetTimeBounds();
      if(timebounds[1]<ScalarTypeNumericTraits::max())
      {
        mitk::ScalarType later = (timebounds[1]-timebounds[0])*t;
        timebounds[0]+=later; timebounds[1]+=later;
        requestedgeometry->SetTimeBounds(timebounds);
      }

      geometry3d = requestedgeometry;
      m_Geometry3Ds[t] = geometry3d;
    }
  }
  else
    return NULL;
  return geometry3d;
}

bool mitk::TimeSlicedGeometry::SetGeometry3D(mitk::Geometry3D* geometry3D, int t)
{
  if(IsValidTime(t))
  {
    m_Geometry3Ds[t]=geometry3D;
    return true;
  }
  return false;
}

int mitk::TimeSlicedGeometry::MSToTimeStep(mitk::ScalarType time_in_ms) const
{
  if(time_in_ms < m_TimeBounds[0])
    return -1;
  if(time_in_ms >= m_TimeBounds[1])
    return m_TimeSteps;
  if(m_EvenlyTimed)
  {
    if(m_TimeBounds[0] == m_TimeBounds[1])
      return 0;
    if((m_TimeBounds[0]>ScalarTypeNumericTraits::NonpositiveMin()) && (m_TimeBounds[1]<ScalarTypeNumericTraits::max()))
      return (int) ((time_in_ms - m_TimeBounds[0])/(m_TimeBounds[1]-m_TimeBounds[0])*m_TimeSteps);
    return 0;
  }
  else
  {
    Point3D projectedPoint;
    unsigned int t;
    for ( t = 0; t < m_TimeSteps; ++t )
    {
      const TimeBounds& timeBounds = GetGeometry3D( t )->GetTimeBounds();
      if( (timeBounds[0] <= time_in_ms) && (time_in_ms <= timeBounds[1]) )
      {
        return t;
      }
    }
  }
  return 0;
}

mitk::ScalarType mitk::TimeSlicedGeometry::TimeStepToMS(int timestep) const
{
  if(IsValidTime(timestep)==false)
    return ScalarTypeNumericTraits::max();
  if(m_EvenlyTimed)
  {
    return ((mitk::ScalarType)timestep)/m_TimeSteps*(m_TimeBounds[1]-m_TimeBounds[0])+m_TimeBounds[0];
  }
  else
  {
    return GetGeometry3D(timestep)->GetTimeBounds()[0];
  }
}

void mitk::TimeSlicedGeometry::Initialize(unsigned int timeSteps)
{
  Geometry3D::Pointer geometry3D = Geometry3D::New();
  geometry3D->Initialize();
  InitializeEvenlyTimed(geometry3D, timeSteps);
}

void mitk::TimeSlicedGeometry::InitializeEvenlyTimed(mitk::Geometry3D* geometry3D, unsigned int timeSteps)
{
  assert(geometry3D!=NULL);

  geometry3D->Register();

  InitializeEmpty(timeSteps);

  AffineTransform3D::Pointer transform = AffineTransform3D::New();
  transform->SetMatrix(geometry3D->GetIndexToWorldTransform()->GetMatrix());
  transform->SetOffset(geometry3D->GetIndexToWorldTransform()->GetOffset());
  SetIndexToWorldTransform(transform);

  SetBounds(geometry3D->GetBounds());
  SetGeometry3D(geometry3D, 0);
  SetEvenlyTimed();

  UpdateInformation();

  SetFrameOfReferenceID(geometry3D->GetFrameOfReferenceID());
  SetImageGeometry(geometry3D->GetImageGeometry());

  geometry3D->UnRegister();
}

void mitk::TimeSlicedGeometry::InitializeEmpty(unsigned int timeSteps)
{
  m_IndexToWorldTransform = NULL;

  Superclass::Initialize();

  m_TimeSteps = timeSteps;
  
  // initialize with empty geometries
  Geometry3D::Pointer gnull=NULL;
  m_Geometry3Ds.assign(m_TimeSteps, gnull);
}

void mitk::TimeSlicedGeometry::ResizeToNumberOfTimeSteps( unsigned int timeSteps)
{
  if( timeSteps <= m_TimeSteps ) return;

  // Resize to Number of timesteps; initialize with empty geometries
  Geometry3D::Pointer gnull=NULL;
  m_Geometry3Ds.resize(timeSteps, gnull);

  m_TimeSteps = timeSteps;

  UpdateInformation();
}

mitk::TimeSlicedGeometry::TimeSlicedGeometry() : m_TimeSteps(0), m_EvenlyTimed(false)
{
}

mitk::TimeSlicedGeometry::~TimeSlicedGeometry()
{

}

void mitk::TimeSlicedGeometry::SetImageGeometry(const bool isAnImageGeometry)
{
  Superclass::SetImageGeometry(isAnImageGeometry);

  mitk::Geometry3D* geometry3d;
  unsigned int t;
  for(t=0; t<m_TimeSteps; ++t)
  {
    geometry3d = m_Geometry3Ds[t];
    if(geometry3d!=NULL)
      geometry3d->SetImageGeometry(isAnImageGeometry);
  }
}

void mitk::TimeSlicedGeometry::SetEvenlyTimed(bool on)
{
  m_EvenlyTimed = on;
  Modified();
}

bool mitk::TimeSlicedGeometry::IsValidTime(int t) const
{
  return (t>=0) && (t< (int)m_TimeSteps);
}

void mitk::TimeSlicedGeometry::CopyTimes(const mitk::TimeSlicedGeometry* timeslicedgeometry, unsigned int t, unsigned int endtimeindex)
{
  if(endtimeindex >= timeslicedgeometry->GetTimeSteps())
    endtimeindex = timeslicedgeometry->GetTimeSteps()-1;
  if(endtimeindex >= this->GetTimeSteps())
    endtimeindex = this->GetTimeSteps()-1;
  for(; t <= endtimeindex; ++t)
  {
    mitk::Geometry3D* geometry3d = GetGeometry3D(t);
    mitk::Geometry3D* othergeometry3d = timeslicedgeometry->GetGeometry3D(t);
    assert((geometry3d!=NULL) && (othergeometry3d!=NULL));

    geometry3d->SetTimeBounds(othergeometry3d->GetTimeBounds());

  }

  UpdateInformation();
}

mitk::AffineGeometryFrame3D::Pointer mitk::TimeSlicedGeometry::Clone() const
{
  Self::Pointer newGeometry = Self::New();
  newGeometry->Initialize(m_TimeSteps);
  InitializeGeometry(newGeometry);
  return newGeometry.GetPointer();
}

void mitk::TimeSlicedGeometry::InitializeGeometry(Self * newGeometry) const
{
  Superclass::InitializeGeometry(newGeometry);

  newGeometry->SetEvenlyTimed(m_EvenlyTimed);

  unsigned int t;
  for(t=0; t<m_TimeSteps; ++t)
  {
    if(m_Geometry3Ds[t].IsNull())
    {
      assert(m_EvenlyTimed);
    }
    else
    {
      newGeometry->SetGeometry3D(dynamic_cast<Geometry3D*>(m_Geometry3Ds[t]->Clone().GetPointer()), t);
    }
  }
}

void mitk::TimeSlicedGeometry::PrintSelf(std::ostream& os, itk::Indent indent) const
{
  Superclass::PrintSelf(os,indent);
  os << indent << " EvenlyTimed: " << m_EvenlyTimed << std::endl;
  os << indent << " TimeSteps: " << m_TimeSteps << std::endl;

  os << indent << " GetGeometry3D(0): ";
  if(GetGeometry3D(0)==NULL)
    os << "NULL" << std::endl;
  else
    GetGeometry3D(0)->Print(os, indent);
}


bool mitk::TimeSlicedGeometry::WriteXMLData( XMLWriter& xmlWriter )
{
  Geometry3D::WriteXMLData( xmlWriter );

  xmlWriter.WriteProperty( EVENLY_TIMED, m_EvenlyTimed );

  xmlWriter.WriteProperty( TIME_STEPS, (int) m_TimeSteps );
 
  std::vector<Geometry3D::Pointer>::iterator i = m_Geometry3Ds.begin();
  const std::vector<Geometry3D::Pointer>::iterator end = m_Geometry3Ds.end();
  
  while ( i != end )
  {
    (*i)->WriteXML( xmlWriter );
    i++;
  }

  return true;
}

bool mitk::TimeSlicedGeometry::ReadXMLData( XMLReader& xmlReader )
{ 
  Geometry3D::ReadXMLData( xmlReader );
  xmlReader.GetAttribute( EVENLY_TIMED, m_EvenlyTimed ); 
  int timeSteps = -1;
  xmlReader.GetAttribute( TIME_STEPS, timeSteps );
  m_TimeSteps = timeSteps;

  if ( m_EvenlyTimed )
  {
    if ( xmlReader.Goto( Geometry3D::XML_NODE_NAME ) ) {
      Geometry3D::Pointer geometry3D = dynamic_cast<mitk::Geometry3D*>( xmlReader.CreateObject().GetPointer() );
      if ( geometry3D.IsNotNull() ) geometry3D->ReadXMLData( xmlReader );
      InitializeEvenlyTimed( geometry3D, GetTimeSteps() );
      xmlReader.GotoParent();
    }
  } else if ( xmlReader.Goto( Geometry3D::XML_NODE_NAME ) ) {

    do {
      Geometry3D::Pointer geometry3D = dynamic_cast<mitk::Geometry3D*>( xmlReader.CreateObject().GetPointer() );
      if ( geometry3D.IsNotNull() ) geometry3D->ReadXMLData( xmlReader );
      m_Geometry3Ds.push_back( geometry3D );
    } while ( xmlReader.GotoNext() );
    xmlReader.GotoParent();
  }

  return true;
}

void mitk::TimeSlicedGeometry::ExecuteOperation(Operation* operation)
{
  // reach through to all time steps
  for (std::vector<Geometry3D::Pointer>::iterator iter = m_Geometry3Ds.begin();
       iter != m_Geometry3Ds.end();
       ++iter)
  {
    (*iter)->ExecuteOperation(operation);
  }
    
  Geometry3D::ExecuteOperation(operation);

  this->Modified();
}

