
#ifndef PROXY_ALGORITHM
#define PROXY_ALGORITHM

#include <windows.h>
#include <GL/GL.h>

#include <chai3d/chai3d.h>
#include <chai3d/forces/CProxyPointForceAlgo.h>

class ProxyPointForceAlgoWrapper;

// Our own Proxy Algorithm using the wrapper around CHAI3D Proxy force algorithm
class ProxyAlgorithm
{

public:

   ProxyAlgorithm(ProxyPointForceAlgoWrapper* wrapper) { m_proxy_wrapper = wrapper; }

   void computeProxyPosition(const cVector3d & goal);

   cVector3d getContactNormal(unsigned int contact_number);
   
   void setProxyPosition(cVector3d new_proxy_pos);

   double getContactObjectStiffness() { return m_object_stiffness; }
   unsigned int getNumContacts() { return m_num_contacts; }
   cVector3d getProxyPosition() { return m_proxy_pos; }
   cVector3d getProxyContactPosition() { return m_proxy_contact_pos; }

protected:

   cVector3d m_proxy_pos;
   cVector3d m_proxy_contact_pos;
   unsigned int m_num_contacts;

   cVector3d m_contact_normals[3];
   double m_object_stiffness;

   ProxyPointForceAlgoWrapper* m_proxy_wrapper;

};

// the wrapper around CHAI3D Proxy force algorithm
class ProxyPointForceAlgoWrapper : public cProxyPointForceAlgo
{

public:

   void computeProxyPosition(const cVector3d & goal, cVector3d & proxyPos, cVector3d & proxyPosBeforeFriction, unsigned int & numContacts, cVector3d contact_normals[3], double & object_stiffness);

   friend class ProxyAlgorithm;

};


#endif // PROXY_ALGORITHM