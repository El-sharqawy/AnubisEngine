#pragma once

#include "Singleton.h"
#include "ServiceLocator.h"
#include <unordered_map>
#include "API/Pipeline.h"

class CPipelinesManager : public CSingleton<CPipelinesManager>
{
public:
	bool Initialize();
	IPipeline* GetOrCreatePipeline(const SPipelineDesc& desc);
	IPipeline* GetPipeline(const EPipelineType& type);
	void Clear(); // destroys all cached pipelines, call on shutdown

	bool CreateStaticMeshPipeline();
	bool CreateSkeletalMeshPipeline();

private:
	std::unordered_map<EPipelineType ,IPipeline*> m_mapPipelines;
};