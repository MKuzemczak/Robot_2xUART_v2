#include "Action.h"

ArchMovAction::ArchMovAction(Eigen::Vector3d start,
							Eigen::Vector3d inter,
							Eigen::Vector3d dest)
{
	starting = start;
	intermediate = inter;
	destination = dest;
	
	setType(ARCH);
}
	
ArchMovAction::~ArchMovAction()
{
	
}

void ArchMovAction::calculate(Robot & robot)
{
	
}
	
void ArchMovAction::execute()
{
	
}