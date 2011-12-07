/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgRef__
#define __dgRef__

//#include "dgStdafx.h"
#include "dgCRC.h"
#include "dgRtti.h"
#include "dgMemory.h"

struct dgRefFlags
{
	dgRefFlags ();
	inline hacd::HaI32 operator = (hacd::HaI32 val);

	hacd::HaU8 m_alive; 
	hacd::HaU8 m_userFlag0;
	hacd::HaU8 m_userFlag1;
	hacd::HaU8 m_userFlag2;
//	hacd::HaU32 m_userFlag3	: 1;
//	hacd::HaU32 m_userFlag4	: 1;
//	hacd::HaU32 m_userFlag5	: 1;
//	hacd::HaU32 m_userFlag6	: 1;

	hacd::HaU32 m_ref;
};


class dgRef: public dgRefFlags
{
   public:
	dgRef ();
	dgRef (const char *name);
	dgRef (hacd::HaU32 idArg);
	dgRef(const dgRef &Clone);
	dgRef *AddRef () ;
	hacd::HaI32 Release ();
	hacd::HaI32 GetRefCount() const;


	virtual dgRef *CreateClone ()	const;
	virtual hacd::HaU32 GetTypeId () const;
	virtual bool IsType (hacd::HaU32 typeId) const;

	bool GetUserFlag0 () const;
	bool GetUserFlag1 () const;
	void SetUserFlag0 (bool flags);
	void SetUserFlag1 (bool flags);

	bool IsAlive() const;
	virtual void Kill(); 
	virtual void Unkill();

	const char* GetName () const;
	hacd::HaU32 GetNameID () const;
	inline void SetNameID (hacd::HaU32 newID);
	virtual inline void SetName (const char *name);

	void AttachRef (dgRef **oldRef, dgRef *newRef);

	
	bool IsTypeByName (const char *typeName) const;
	static hacd::HaU32 GetRttiType();

	protected:
	virtual ~dgRef (); 

	private:
	hacd::HaU32 m_id;
	static dgRtti m_rtti;
};



inline dgRefFlags::dgRefFlags ()
{
	*this = 0;
	m_alive = true;
	m_ref = 1;
}

inline hacd::HaI32 dgRefFlags::operator = (hacd::HaI32 val)
{
	hacd::HaI32* ptr;
	ptr = &(*(hacd::HaI32*)this);
	*ptr = val;
	return val;
}



inline dgRef::dgRef ()
{
	m_id = 0;
}

inline dgRef::dgRef (const char *name)
{
	SetName (name);
}

inline dgRef::dgRef (hacd::HaU32 idArg)
{
	SetNameID (idArg);
}

inline dgRef::dgRef(const dgRef &Clone)
{
	m_id = Clone.m_id;
}

inline dgRef::~dgRef () 
{
}

inline dgRef *dgRef::AddRef () 
{
	m_ref ++;
	HACD_ASSERT (m_ref < ((1<<24) - 1));
	return this;
}

inline hacd::HaI32 dgRef::Release ()
{
	m_ref --;
	if (m_ref) {
		return hacd::HaI32 (m_ref);
	}
	delete this;
	return 0;
}

inline dgRef *dgRef::CreateClone () const
{
	HACD_ASSERT (0);
	return NULL;
}


inline hacd::HaU32 dgRef::GetTypeId () const
{
	return m_rtti.GetTypeId ();
}

inline bool dgRef::IsType (hacd::HaU32 typeId) const
{
	return m_rtti.IsTypeID (typeId);
}

inline hacd::HaU32 dgRef::GetRttiType()
{
	return m_rtti.GetTypeId();
}

inline bool dgRef::IsTypeByName (const char *typeName) const
{
	return IsType (dgCRC (typeName,  (hacd::HaI32) strlen (typeName)));
}


inline bool dgRef::GetUserFlag0 () const
{
	return m_userFlag0 ? true : false;
}

inline bool dgRef::GetUserFlag1 () const
{
	return m_userFlag1 ? true : false;
}


inline void dgRef::SetUserFlag0 (bool flags)
{
	m_userFlag0 = hacd::HaU8 (flags);
}

inline void dgRef::SetUserFlag1 (bool flags)
{
	m_userFlag1 = hacd::HaU8 (flags);
}


inline bool dgRef::IsAlive() const
{
	return m_alive ? true : false;
}

inline void dgRef::Kill() 
{
	m_alive = false;
}

inline void dgRef::Unkill() 
{
	m_alive = true;
}

inline void dgRef::SetNameID (hacd::HaU32 newID)
{
	m_id = newID;
}

inline hacd::HaU32 dgRef::GetNameID () const
{
	return m_id;
}

inline void dgRef::SetName (const char *name)
{
	SetNameID (0);
	if (name) {
		SetNameID (dgCRC (name));
	}
}

inline const char* dgRef::GetName () const
{
	return dgInverseCRC (GetNameID ());
}

inline hacd::HaI32 dgRef::GetRefCount() const
{
	return hacd::HaI32 (m_ref);
}

#endif

