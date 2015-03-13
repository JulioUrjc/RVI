
#include "ProxyAlgorithm.h"

void ProxyAlgorithm::computeProxyPosition(const cVector3d & goal)
{
   m_proxy_wrapper->computeProxyPosition(goal, m_proxy_pos, m_proxy_contact_pos, m_num_contacts, m_contact_normals, m_object_stiffness);

}

cVector3d ProxyAlgorithm::getContactNormal(unsigned int contact_number)
{
   // if out of range (not between 0 and 2)
   if (contact_number > 2)
   {
      std::cout << "Contact number out of range. Should be between 0 and 2." << std::endl;
      return cVector3d(0.,0.,0.);
   }

   return m_contact_normals[contact_number];
}

void ProxyAlgorithm::setProxyPosition(cVector3d new_proxy_pos) 
{
   m_proxy_wrapper->m_proxyGlobalPos = new_proxy_pos;
   m_proxy_pos = new_proxy_pos;
}


void ProxyPointForceAlgoWrapper::computeProxyPosition(const cVector3d & goal, cVector3d & proxyPos, cVector3d & proxyPosBeforeFriction, unsigned int & numContacts, cVector3d contact_normals[3], double & object_stiffness)
{
   // compute positions with up to 3 contacts
   bool hit0, hit1, hit2;
   // first contact
   hit0 = this->computeNextProxyPositionWithContraints0(goal);
   this->m_proxyGlobalPos = this->m_nextBestProxyGlobalPos;
   proxyPosBeforeFriction = this->m_nextBestProxyGlobalPos;
   if (hit0)
   {
      // second contact
      hit1 = this->computeNextProxyPositionWithContraints1(goal);
      this->m_proxyGlobalPos = this->m_nextBestProxyGlobalPos;
      if (hit1)
      {
         // third contact
         hit2 = this->computeNextProxyPositionWithContraints2(goal);
         this->m_proxyGlobalPos = this->m_nextBestProxyGlobalPos;
      }
   }

   proxyPos = this->m_proxyGlobalPos;
   numContacts = this->m_numContacts;

   object_stiffness = 0.;
   for (unsigned int i=0; i<3; i++)
   {
      contact_normals[i] = cVector3d(0.,0.,0.);
   }
      
   if (numContacts > 0)
   {
      contact_normals[0] = this->m_contactPoint0->m_globalNormal;
      object_stiffness = this->m_contactPoint0->m_triangle->getParent()->m_material.getStiffness();
   }
   if (numContacts > 1)
   {
      contact_normals[1] = this->m_contactPoint1->m_globalNormal;
   }
   if (numContacts > 2)
   {
      contact_normals[2] = this->m_contactPoint2->m_globalNormal;
   }
}
