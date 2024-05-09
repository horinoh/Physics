#include "VK.h"

#pragma comment(lib, "vulkan-1.lib")
#define VK_PROC_ADDR(proc) PFN_vk ## proc VK::vk ## proc = VK_NULL_HANDLE;
#include "VKGlobalProcAddr.h"
#include "VKInstanceProcAddr.h"
#include "VKDeviceProcAddr.h"
#undef VK_PROC_ADDR

template<> const glm::vec3 GetMin(const glm::vec3& lhs, const glm::vec3& rhs) { return glm::vec3((std::min)(lhs.x, rhs.x), (std::min)(lhs.y, rhs.y), (std::min)(lhs.z, rhs.z)); }
template<> const glm::vec3 GetMax(const glm::vec3& lhs, const glm::vec3& rhs) { return glm::vec3((std::max)(lhs.x, rhs.x), (std::max)(lhs.y, rhs.y), (std::max)(lhs.z, rhs.z)); }
template<>
void AdjustScale(std::vector<glm::vec3>& Vertices, const float Scale)
{
	auto Max = (std::numeric_limits<glm::vec3>::min)();
	auto Min = (std::numeric_limits<glm::vec3>::max)();
	for (const auto& i : Vertices) {
		Min = GetMin(Min, i);
		Max = GetMax(Max, i);
	}
	const auto Diff = Max - Min;
	const auto Bound = (std::max)((std::max)(Diff.x, Diff.y), Diff.z);
	const auto Coef = Scale / Bound;
	std::ranges::transform(Vertices, std::begin(Vertices), [&](const glm::vec3& rhs) { return glm::vec3(rhs.x, (rhs.y - Diff.y * 0.5f), rhs.z - Min.z) * Coef; });
}

void VK::OnDestroy(HWND hWnd, HINSTANCE hInstance)
{
	for (auto i : DescriptorUpdateTemplates) {
		vkDestroyDescriptorUpdateTemplate(Device, i, GetAllocationCallbacks());
	}
	DescriptorUpdateTemplates.clear();
	//!< 個別に開放できるのは VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT を指定した場合
	//if (!empty(DescriptorSets)) { vkFreeDescriptorSets(Device, DescriptorPool, static_cast<uint32_t>(size(DescriptorSets)), std::data(DescriptorSets)); }
	DescriptorSets.clear();
	//!< やらなくてもよい
	//for (auto i : DescriptorPools) { vkResetDescriptorPool(Device, i, 0); }
	for (auto i : DescriptorPools) {
		vkDestroyDescriptorPool(Device, i, GetAllocationCallbacks());
	}
	DescriptorPools.clear();

	for (auto i : Framebuffers) {
		vkDestroyFramebuffer(Device, i, GetAllocationCallbacks());
	}
	Framebuffers.clear();
	for (auto i : Pipelines) {
		vkDestroyPipeline(Device, i, GetAllocationCallbacks());
	}
	Pipelines.clear();
	for (auto i : RenderPasses) {
		vkDestroyRenderPass(Device, i, GetAllocationCallbacks());
	}
	RenderPasses.clear();

	for (auto i : PipelineLayouts) {
		vkDestroyPipelineLayout(Device, i, GetAllocationCallbacks());
	}
	PipelineLayouts.clear();
	for (auto i : DescriptorSetLayouts) {
		vkDestroyDescriptorSetLayout(Device, i, GetAllocationCallbacks());
	}
	DescriptorSetLayouts.clear();

	for (auto& i : DepthTextures) {
		i.Destroy(Device);
	}
	DepthTextures.clear();

	for (auto i : UniformBuffers) { 
		i.Destroy(Device);
	}
	UniformBuffers.clear();

	for (auto i : IndirectBuffers) { 
		i.Destroy(Device); 
	}
	IndirectBuffers.clear();
	for (auto i : IndexBuffers) {
		i.Destroy(Device);
	}
	IndexBuffers.clear();
	for (auto i : VertexBuffers) {
		i.Destroy(Device);
	}
	VertexBuffers.clear();

	//!< コマンドプール破棄時にコマンドバッファは暗黙的に破棄される
	for (auto i : SecondaryCommandPools) {
		vkDestroyCommandPool(Device, i, GetAllocationCallbacks());
	}
	SecondaryCommandPools.clear();
	for (auto i : CommandPools) {
		vkDestroyCommandPool(Device, i, GetAllocationCallbacks());
	}
	CommandPools.clear();

	for (auto& i : SwapchainBackBuffers) {
		vkDestroyImageView(Device, i.ImageView, GetAllocationCallbacks());
	} 
	SwapchainBackBuffers.clear();
	if (VK_NULL_HANDLE != Swapchain) [[likely]] { 
		vkDestroySwapchainKHR(Device, Swapchain, GetAllocationCallbacks()); 
		Swapchain = VK_NULL_HANDLE;
	}

	if (VK_NULL_HANDLE != RenderFinishedSemaphore) [[likely]] {
		vkDestroySemaphore(Device, RenderFinishedSemaphore, GetAllocationCallbacks());
	}
	if (VK_NULL_HANDLE != NextImageAcquiredSemaphore) [[likely]] {
		vkDestroySemaphore(Device, NextImageAcquiredSemaphore, GetAllocationCallbacks());
	}
	if (VK_NULL_HANDLE != GraphicsFence) [[likely]] {
		vkDestroyFence(Device, GraphicsFence, GetAllocationCallbacks());
	}

	if (VK_NULL_HANDLE != Device) [[likely]] {
		vkDestroyDevice(Device, GetAllocationCallbacks());
		Device = VK_NULL_HANDLE;
	}
	if (VK_NULL_HANDLE != Surface) [[likely]] {
		vkDestroySurfaceKHR(Instance, Surface, GetAllocationCallbacks());
		Surface = VK_NULL_HANDLE;
	}
	if (VK_NULL_HANDLE != Instance) [[likely]] {
		vkDestroyInstance(Instance, GetAllocationCallbacks());
		Instance = VK_NULL_HANDLE;
	}
	if (!FreeLibrary(VulkanLibrary)) [[likely]] { VulkanLibrary = nullptr; }
}

void VK::LoadVulkanLibrary() 
{
	VulkanLibrary = LoadLibrary(TEXT("vulkan-1.dll"));

#define VK_PROC_ADDR(proc) vk ## proc = reinterpret_cast<PFN_vk ## proc>(vkGetInstanceProcAddr(nullptr, "vk" #proc)); VERIFY(nullptr != vk ## proc && #proc);
#include "VKGlobalProcAddr.h"
#undef VK_PROC_ADDR
}

void VK::CreateInstance(const std::vector<const char*>& AdditionalLayers, const std::vector<const char*>& AdditionalExtensions)
{
	constexpr auto APIVersion = VK_HEADER_VERSION_COMPLETE;
	const VkApplicationInfo AI = {
		.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
		.pNext = nullptr,
		.pApplicationName = "VK", .applicationVersion = APIVersion,
		.pEngineName = "VK Engine Name", .engineVersion = APIVersion,
		.apiVersion = APIVersion
	};

	//!< レイヤ
	std::vector Layers = {
		"VK_LAYER_KHRONOS_validation",
		"VK_LAYER_LUNARG_monitor", //!< タイトルバーにFPSを表示 (Display FPS on titile bar)
	};
	std::ranges::copy(AdditionalLayers, std::back_inserter(Layers));

	//!< エクステンション
	std::vector Extensions = {
	VK_KHR_SURFACE_EXTENSION_NAME,
#ifdef VK_USE_PLATFORM_WIN32_KHR
		VK_KHR_WIN32_SURFACE_EXTENSION_NAME,
#endif
#ifdef _DEBUG
		VK_EXT_DEBUG_UTILS_EXTENSION_NAME,
		VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME,
#endif
	};
	std::ranges::copy(AdditionalExtensions, std::back_inserter(Extensions));

	//!< インスタンスの作成
	const VkInstanceCreateInfo ICI = {
		.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.pApplicationInfo = &AI,
		.enabledLayerCount = static_cast<uint32_t>(std::size(Layers)), .ppEnabledLayerNames = std::data(Layers),
		.enabledExtensionCount = static_cast<uint32_t>(std::size(Extensions)), .ppEnabledExtensionNames = std::data(Extensions)
	};
	VERIFY_SUCCEEDED(vkCreateInstance(&ICI, GetAllocationCallbacks(), &Instance));

#define VK_PROC_ADDR(proc) vk ## proc = reinterpret_cast<PFN_vk ## proc>(vkGetInstanceProcAddr(Instance, "vk" #proc)); VERIFY(nullptr != vk ## proc && #proc);
#include "VKInstanceProcAddr.h"
#undef VK_PROC_ADDR
}

void VK::SelectPhysicalDevice(VkInstance Inst)
{
	//!< 物理デバイス(GPU)の列挙
	uint32_t Count = 0;
	VERIFY_SUCCEEDED(vkEnumeratePhysicalDevices(Inst, &Count, nullptr));
	PhysicalDevices.resize(Count);
	VERIFY_SUCCEEDED(vkEnumeratePhysicalDevices(Inst, &Count, std::data(PhysicalDevices)));

	//!< 物理デバイスの選択、ここでは最大メモリを選択することにする (Select physical device, here select max memory size)
	const auto Index = std::distance(begin(PhysicalDevices), std::ranges::max_element(PhysicalDevices, [](const VkPhysicalDevice& lhs, const VkPhysicalDevice& rhs) {
		std::array<VkPhysicalDeviceMemoryProperties, 2> PDMPs;
		vkGetPhysicalDeviceMemoryProperties(lhs, &PDMPs[0]);
		vkGetPhysicalDeviceMemoryProperties(rhs, &PDMPs[1]);
		return std::accumulate(&PDMPs[0].memoryHeaps[0], &PDMPs[0].memoryHeaps[PDMPs[0].memoryHeapCount], static_cast<VkDeviceSize>(0), [](VkDeviceSize Sum, const VkMemoryHeap& rhs) { return Sum + rhs.size; })
			< std::accumulate(&PDMPs[1].memoryHeaps[0], &PDMPs[1].memoryHeaps[PDMPs[1].memoryHeapCount], static_cast<VkDeviceSize>(0), [](VkDeviceSize Sum, const VkMemoryHeap& rhs) { return Sum + rhs.size; });
		}));
	CurrentPhysicalDevice = PhysicalDevices[Index];
	vkGetPhysicalDeviceMemoryProperties(CurrentPhysicalDevice, &CurrentPhysicalDeviceMemoryProperties);
}
void VK::CreateDevice(HWND hWnd, HINSTANCE hInstance, void* pNext, const std::vector<const char*>& AdditionalExtensions)
{
	const VkWin32SurfaceCreateInfoKHR SCI = {
		.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR,
		.pNext = nullptr,
		.flags = 0,
		.hinstance = hInstance,
		.hwnd = hWnd
	};
	VERIFY_SUCCEEDED(vkCreateWin32SurfaceKHR(Instance, &SCI, GetAllocationCallbacks(), &Surface));

	const auto PD = CurrentPhysicalDevice;

	std::vector<VkQueueFamilyProperties> QFPs;
	{
		uint32_t Count = 0;
		vkGetPhysicalDeviceQueueFamilyProperties(PD, &Count, nullptr);
		QFPs.resize(Count);
		vkGetPhysicalDeviceQueueFamilyProperties(PD, &Count, std::data(QFPs));

		{
			constexpr uint8_t QueueFamilyPropMax = 8;
			VERIFY(size(QFPs) <= QueueFamilyPropMax);
			//!< 機能を持つキューファミリインデックスのビットを立てておく
			std::bitset<QueueFamilyPropMax> GraphicsMask;
			std::bitset<QueueFamilyPropMax> PresentMask;
			//!< 最初に見つかった、機能を持つキューファミリインデックス
			GraphicsQueueFamilyIndex = UINT32_MAX;
			PresentQueueFamilyIndex = UINT32_MAX;
			for (auto i = 0; i < size(QFPs); ++i) {
				if (VK_QUEUE_GRAPHICS_BIT & QFPs[i].queueFlags) {
					GraphicsMask.set(i);
					if (UINT32_MAX == GraphicsQueueFamilyIndex) {
						GraphicsQueueFamilyIndex = i;
					}
				}
				VkBool32 b = VK_FALSE;
				VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceSupportKHR(PD, i, Surface, &b));
				if (b) {
					PresentMask.set(i);
					if (UINT32_MAX == PresentQueueFamilyIndex) {
						PresentQueueFamilyIndex = i;
					}
				}
				if (VK_QUEUE_COMPUTE_BIT & QFPs[i].queueFlags) {}
				if (VK_QUEUE_TRANSFER_BIT & QFPs[i].queueFlags) {}
				if (VK_QUEUE_SPARSE_BINDING_BIT & QFPs[i].queueFlags) {}
				if (VK_QUEUE_PROTECTED_BIT & QFPs[i].queueFlags) {}
			}

			//!< キューファミリ内でのインデックス及びプライオリティ、ここではグラフィック、プレゼントの分をプライオリティ0.5fで追加している
			std::vector<std::vector<float>> Priorites(size(QFPs));
			const uint32_t GraphicsQueueIndexInFamily = static_cast<uint32_t>(size(Priorites[GraphicsQueueFamilyIndex])); Priorites[GraphicsQueueFamilyIndex].emplace_back(0.5f);
			const uint32_t PresentQueueIndexInFamily = static_cast<uint32_t>(size(Priorites[PresentQueueFamilyIndex])); Priorites[PresentQueueFamilyIndex].emplace_back(0.5f);
		
			//!< キュー作成情報 (Queue create information)
			std::vector<VkDeviceQueueCreateInfo> DQCIs;
			for (size_t i = 0; i < size(Priorites); ++i) {
				if (!empty(Priorites[i])) {
					DQCIs.emplace_back(
						VkDeviceQueueCreateInfo({
							.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
							.pNext = nullptr,
							.flags = 0,
							.queueFamilyIndex = static_cast<uint32_t>(i),
							.queueCount = static_cast<uint32_t>(std::size(Priorites[i])), .pQueuePriorities = std::data(Priorites[i])
							})
					);
				}
			}

			std::vector Extensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
			std::ranges::copy(AdditionalExtensions, std::back_inserter(Extensions));

			VkPhysicalDeviceFeatures PDF;
			vkGetPhysicalDeviceFeatures(PD, &PDF);
			if (nullptr == pNext) {
				const VkDeviceCreateInfo DCI = {
					.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
					.pNext = nullptr,
					.flags = 0,
					.queueCreateInfoCount = static_cast<uint32_t>(std::size(DQCIs)), .pQueueCreateInfos = std::data(DQCIs),
					.enabledLayerCount = 0, .ppEnabledLayerNames = nullptr,
					.enabledExtensionCount = static_cast<uint32_t>(std::size(Extensions)), .ppEnabledExtensionNames = std::data(Extensions),
					.pEnabledFeatures = &PDF
				};
				VERIFY_SUCCEEDED(vkCreateDevice(PD, &DCI, GetAllocationCallbacks(), &Device));
			}

#define VK_PROC_ADDR(proc) vk ## proc = reinterpret_cast<PFN_vk ## proc>(vkGetDeviceProcAddr(Device, "vk" #proc)); VERIFY(nullptr != vk ## proc && #proc && #proc);
#include "VKDeviceProcAddr.h"
#undef VK_PROC_ADDR

			vkGetDeviceQueue(Device, GraphicsQueueFamilyIndex, GraphicsQueueIndexInFamily, &GraphicsQueue);
			vkGetDeviceQueue(Device, PresentQueueFamilyIndex, PresentQueueIndexInFamily, &PresentQueue);
		}
	}
}

VkSurfaceFormatKHR VK::SelectSurfaceFormat(VkPhysicalDevice PD, VkSurfaceKHR Sfc)
{
	uint32_t Count = 0;
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceFormatsKHR(PD, Sfc, &Count, nullptr));
	std::vector<VkSurfaceFormatKHR> SFs(Count);
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceFormatsKHR(PD, Sfc, &Count, std::data(SFs)));

	const auto SelectedIndex = [&]() {
		//!< 要素が 1 つのみで UNDEFINED の場合、制限は無く好きなものを選択できる (If there is only 1 element and which is UNDEFINED, we can choose any)
		if (1 == size(SFs) && VK_FORMAT_UNDEFINED == SFs[0].format) {
			return -1;
		}
		for (auto i = 0; i < size(SFs); ++i) {
			//!< VK_FORMAT_UNDEFINED でない最初のもの
			if (VK_FORMAT_UNDEFINED != SFs[i].format) {
				return i;
			}
		}
		//!< ここに来てはいけない
		VERIFY(false);
		return 0;
	}();

	return -1 == SelectedIndex ? VkSurfaceFormatKHR({ VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR }) : SFs[SelectedIndex];
}
VkPresentModeKHR VK::SelectSurfacePresentMode(VkPhysicalDevice PD, VkSurfaceKHR Sfc)
{
	uint32_t Count;
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfacePresentModesKHR(PD, Sfc, &Count, nullptr));
	std::vector<VkPresentModeKHR> PMs(Count);
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfacePresentModesKHR(PD, Sfc, &Count, std::data(PMs)));

	//!< 可能なら VK_PRESENT_MODE_MAILBOX_KHR を選択、そうでなければ VK_PRESENT_MODE_FIFO_KHR を選択 (Want to select VK_PRESENT_MODE_MAILBOX_KHR, or select VK_PRESENT_MODE_FIFO_KHR)
	const auto SelectedPresentMode = [&]() {
		for (auto i : PMs) {
			if (VK_PRESENT_MODE_MAILBOX_KHR == i) {
				//!< 可能なら MAILBOX (If possible, want to use MAILBOX)
				return i;
			}
		}
		for (auto i : PMs) {
			if (VK_PRESENT_MODE_FIFO_KHR == i) {
				//!< FIFO は VulkanAPI が必ずサポートする (VulkanAPI always support FIFO)
				return i;
			}
		}
		VERIFY(false);
		return PMs[0];
	}();

	return SelectedPresentMode;
}

void VK::CreateSwapchain(VkPhysicalDevice PD, VkSurfaceKHR Sfc, const uint32_t Width, const uint32_t Height, const VkImageUsageFlags AdditionalUsage) 
{
	VkSurfaceCapabilitiesKHR SC;
	VERIFY_SUCCEEDED(vkGetPhysicalDeviceSurfaceCapabilitiesKHR(PD, Sfc, &SC));

	//!< 最低よりも1枚多く取りたい、ただし最大値でクランプする(maxImageCount が0の場合は上限無し)
	const auto ImageCount = (std::min)(SC.minImageCount + 1, 0 == SC.maxImageCount ? UINT32_MAX : SC.maxImageCount);

	//!< サーフェスのフォーマットを選択
	const auto SurfaceFormat = SelectSurfaceFormat(PD, Sfc);
	ColorFormat = SurfaceFormat.format; //!< カラーファーマットは覚えておく

	//!< サーフェスのサイズを選択
	//!< currentExtent.width == 0xffffffff の場合はスワップチェインのサイズから決定する (If 0xffffffff, surface size will be determined by the extent of a swapchain targeting the surface)
	SurfaceExtent2D = 0xffffffff != SC.currentExtent.width ? SC.currentExtent : VkExtent2D({ .width = (std::clamp)(Width, SC.maxImageExtent.width, SC.minImageExtent.width), .height = (std::clamp)(Height, SC.minImageExtent.height, SC.minImageExtent.height) });

	//!< レイヤー、ステレオレンダリング等をしたい場合は1以上になるが、ここでは1
	uint32_t ImageArrayLayers = 1;

	VERIFY(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT & SC.supportedUsageFlags);

	//!< グラフィックとプレゼントのキューファミリが異なる場合はキューファミリインデックスの配列が必要、また VK_SHARING_MODE_CONCURRENT を指定すること
	//!< (ただし VK_SHARING_MODE_CONCURRENT にするとパフォーマンスが落ちる場合がある)
	std::vector<uint32_t> QueueFamilyIndices;
	if (GraphicsQueueFamilyIndex != PresentQueueFamilyIndex) {
		QueueFamilyIndices.emplace_back(GraphicsQueueFamilyIndex);
		QueueFamilyIndices.emplace_back(PresentQueueFamilyIndex);
	}

	//!< サーフェスを回転、反転等させるかどうか (Rotate, mirror surface or not)
	const auto SurfaceTransform = (VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR & SC.supportedTransforms) ? VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR : SC.currentTransform;

	//!< サーフェスのプレゼントモードを選択
	const auto SurfacePresentMode = SelectSurfacePresentMode(PD, Sfc);

	//!< 既存のは後で開放するので OldSwapchain に覚えておく (セッティングを変更してスワップチェインを再作成する場合等に備える)
	auto OldSwapchain = Swapchain;
	const VkSwapchainCreateInfoKHR SCI = {
		.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
		.pNext = nullptr,
		.flags = 0,
		.surface = Sfc,
		.minImageCount = ImageCount,
		.imageFormat = SurfaceFormat.format, .imageColorSpace = SurfaceFormat.colorSpace,
		.imageExtent = SurfaceExtent2D,
		.imageArrayLayers = ImageArrayLayers,
		.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | AdditionalUsage,
		.imageSharingMode = empty(QueueFamilyIndices) ? VK_SHARING_MODE_EXCLUSIVE : VK_SHARING_MODE_CONCURRENT,
		.queueFamilyIndexCount = static_cast<uint32_t>(std::size(QueueFamilyIndices)), .pQueueFamilyIndices = std::data(QueueFamilyIndices),
		.preTransform = SurfaceTransform,
		.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
		.presentMode = SurfacePresentMode,
		.clipped = VK_TRUE,
		.oldSwapchain = OldSwapchain
	};
	VERIFY_SUCCEEDED(vkCreateSwapchainKHR(Device, &SCI, GetAllocationCallbacks(), &Swapchain));

	//!< (あれば)前のやつは破棄
	if (VK_NULL_HANDLE != OldSwapchain) {
		vkDestroySwapchainKHR(Device, OldSwapchain, GetAllocationCallbacks());
	}
}
void VK::GetSwapchainImages()
{
	for (auto i : SwapchainBackBuffers) {
		vkDestroyImageView(Device, i.ImageView, GetAllocationCallbacks());
	}
	SwapchainBackBuffers.clear();

	uint32_t Count;
	VERIFY_SUCCEEDED(vkGetSwapchainImagesKHR(Device, Swapchain, &Count, nullptr));
	std::vector<VkImage> Images(Count);
	VERIFY_SUCCEEDED(vkGetSwapchainImagesKHR(Device, Swapchain, &Count, std::data(Images)));
	for (auto i : Images) {
		auto& SBB = SwapchainBackBuffers.emplace_back(SwapchainBackBuffer(i));
		const VkImageViewCreateInfo IVCI = {
			.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
			.pNext = nullptr,
			.flags = 0,
			.image = i,
			.viewType = VK_IMAGE_VIEW_TYPE_2D,
			.format = ColorFormat,
			.components = VkComponentMapping({.r = VK_COMPONENT_SWIZZLE_IDENTITY, .g = VK_COMPONENT_SWIZZLE_IDENTITY, .b = VK_COMPONENT_SWIZZLE_IDENTITY, .a = VK_COMPONENT_SWIZZLE_IDENTITY, }),
			.subresourceRange = VkImageSubresourceRange({.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, .baseMipLevel = 0, .levelCount = 1, .baseArrayLayer = 0, .layerCount = 1 })
		};
		VERIFY_SUCCEEDED(vkCreateImageView(Device, &IVCI, GetAllocationCallbacks(), &SBB.ImageView));
	}
}

void VK::AllocatePrimaryCommandBuffer(const size_t Num)
{
	const VkCommandPoolCreateInfo CPCI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		.pNext = nullptr,
		.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
		.queueFamilyIndex = GraphicsQueueFamilyIndex
	};
	VERIFY_SUCCEEDED(vkCreateCommandPool(Device, &CPCI, GetAllocationCallbacks(), &CommandPools.emplace_back()));

	const auto CP = CommandPools[0];
	CommandBuffers.resize(Num);
	const VkCommandBufferAllocateInfo CBAI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.pNext = nullptr,
		.commandPool = CP,
		.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
		.commandBufferCount = static_cast<uint32_t>(size(CommandBuffers))
	};
	VERIFY_SUCCEEDED(vkAllocateCommandBuffers(Device, &CBAI, std::data(CommandBuffers)));
}
void VK::AllocateSecondaryCommandBuffer(const size_t Num) 
{
	const VkCommandPoolCreateInfo CPCI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
		.pNext = nullptr,
		.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
		.queueFamilyIndex = GraphicsQueueFamilyIndex
	};
	VERIFY_SUCCEEDED(vkCreateCommandPool(Device, &CPCI, GetAllocationCallbacks(), &SecondaryCommandPools.emplace_back()));

	const auto SCP = SecondaryCommandPools[0];
	SecondaryCommandBuffers.resize(Num);
	const VkCommandBufferAllocateInfo CBAI = {
		.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
		.pNext = nullptr,
		.commandPool = SCP,
		.level = VK_COMMAND_BUFFER_LEVEL_SECONDARY,
		.commandBufferCount = static_cast<uint32_t>(size(SecondaryCommandBuffers))
	};
	VERIFY_SUCCEEDED(vkAllocateCommandBuffers(Device, &CBAI, std::data(SecondaryCommandBuffers)));
}

void VK::CreateRenderPass(VkRenderPass& RP, const std::vector<VkAttachmentDescription>& ADs, const std::vector<VkSubpassDescription>& SDs, const std::vector<VkSubpassDependency>& Deps)
{
	const VkRenderPassCreateInfo RPCI = {
		.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.attachmentCount = static_cast<uint32_t>(std::size(ADs)), .pAttachments = std::data(ADs),
		.subpassCount = static_cast<uint32_t>(std::size(SDs)), .pSubpasses = std::data(SDs),
		.dependencyCount = static_cast<uint32_t>(std::size(Deps)), .pDependencies = std::data(Deps)
	};
	VERIFY_SUCCEEDED(vkCreateRenderPass(Device, &RPCI, GetAllocationCallbacks(), &RP));
}

void VK::CreatePipelineVsFsTesTcsGs(VkPipeline& PL,
	const VkDevice Dev,
	const VkPipelineLayout PLL,
	const VkRenderPass RP,
	const VkPrimitiveTopology PT, const uint32_t PatchControlPoints,
	const VkPipelineRasterizationStateCreateInfo& PRSCI,
	const VkPipelineDepthStencilStateCreateInfo& PDSSCI,
	const VkPipelineShaderStageCreateInfo* VS, const VkPipelineShaderStageCreateInfo* FS, const VkPipelineShaderStageCreateInfo* TES, const VkPipelineShaderStageCreateInfo* TCS, const VkPipelineShaderStageCreateInfo* GS,
	const std::vector<VkVertexInputBindingDescription>& VIBDs, const std::vector<VkVertexInputAttributeDescription>& VIADs,
	const std::vector<VkPipelineColorBlendAttachmentState>& PCBASs)
{
	//!< シェーダステージ (ShaderStage)
	std::vector<VkPipelineShaderStageCreateInfo> PSSCIs;
	if (nullptr != VS) { PSSCIs.emplace_back(*VS); }
	if (nullptr != FS) { PSSCIs.emplace_back(*FS); }
	if (nullptr != TES) { PSSCIs.emplace_back(*TES); }
	if (nullptr != TCS) { PSSCIs.emplace_back(*TCS); }
	if (nullptr != GS) { PSSCIs.emplace_back(*GS); }
	VERIFY(!empty(PSSCIs));

	//!< バーテックスインプット (VertexInput)
	const VkPipelineVertexInputStateCreateInfo PVISCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.vertexBindingDescriptionCount = static_cast<uint32_t>(std::size(VIBDs)), .pVertexBindingDescriptions = std::data(VIBDs),
		.vertexAttributeDescriptionCount = static_cast<uint32_t>(std::size(VIADs)), .pVertexAttributeDescriptions = std::data(VIADs)
	};

	//!< DXでは「トポロジ」と「パッチコントロールポイント」の指定はIASetPrimitiveTopology()の引数としてコマンドリストへ指定する、VKとは結構異なるので注意
	//!< (「パッチコントロールポイント」の数も何を指定するかにより決まる)
	//!< CommandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	//!< CommandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_1_CONTROL_POINT_PATCHLIST);

	//!< インプットアセンブリ (InputAssembly)
	const VkPipelineInputAssemblyStateCreateInfo PIASCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.topology = PT,
		.primitiveRestartEnable = VK_FALSE
	};
	//!< WITH_ADJACENCY 系使用時には デバイスフィーチャー geometryShader が有効であること
	//assert((
	//	(PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_STRIP_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP_WITH_ADJACENCY)
	//	|| PDF.geometryShader) /*&& ""*/);
	//!< PATCH_LIST 使用時には デバイスフィーチャー tessellationShader が有効であること
	//assert((PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_PATCH_LIST || PDF.tessellationShader) && "");
	//!< インデックス 0xffffffff(VK_INDEX_TYPE_UINT32), 0xffff(VK_INDEX_TYPE_UINT16) をプリミティブのリスタートとする、インデックス系描画の場合(vkCmdDrawIndexed, vkCmdDrawIndexedIndirect)のみ有効
	//!< LIST 系使用時 primitiveRestartEnable 無効であること
	VERIFY((
		(PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_POINT_LIST || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_LIST || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_LINE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST_WITH_ADJACENCY || PIASCI.topology != VK_PRIMITIVE_TOPOLOGY_PATCH_LIST)
		|| PIASCI.primitiveRestartEnable == VK_FALSE));

	//!< テセレーション (Tessellation)
	VERIFY((PT != VK_PRIMITIVE_TOPOLOGY_PATCH_LIST || PatchControlPoints != 0));
	const VkPipelineTessellationStateCreateInfo PTSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.patchControlPoints = PatchControlPoints //!< パッチコントロールポイント
	};

	//!< ビューポート (Viewport)
	//!< VkDynamicState を使用するため、ここではビューポート(シザー)の個数のみ指定している (To use VkDynamicState, specify only count of viewport(scissor) here)
	//!< 後に vkCmdSetViewport(), vkCmdSetScissor() で指定する (Use vkCmdSetViewport(), vkCmdSetScissor() later)
	constexpr VkPipelineViewportStateCreateInfo PVSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.viewportCount = 1, .pViewports = nullptr,
		.scissorCount = 1, .pScissors = nullptr
	};
	//!< 2つ以上のビューポートを使用するにはデバイスフィーチャー multiViewport が有効であること (If use 2 or more viewport device feature multiViewport must be enabled)
	//!< ビューポートのインデックスはジオメトリシェーダで指定する (Viewport index is specified in geometry shader)
	//assert((PVSCI.viewportCount <= 1 || PDF.multiViewport) && "");

	//!< PRSCI
	//!< FILL以外使用時には、デバイスフィーチャーfillModeNonSolidが有効であること
	//assert(PRSCI.polygonMode == VK_POLYGON_MODE_FILL || PDF.fillModeNonSolid && "");
	//!< 1.0f より大きな値には、デバイスフィーチャーwidelines が有効であること
	//assert(PRSCI.lineWidth <= 1.0f || PDF.wideLines && "");

	//!< マルチサンプル (Multisample)
	constexpr VkSampleMask SM = 0xffffffff; //!< 0xffffffff を指定する場合は、代わりに nullptr でもよい
	const VkPipelineMultisampleStateCreateInfo PMSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT,
		.sampleShadingEnable = VK_FALSE, .minSampleShading = 0.0f,
		.pSampleMask = &SM,
		.alphaToCoverageEnable = VK_FALSE, .alphaToOneEnable = VK_FALSE
	};
	//assert((PMSCI.sampleShadingEnable == VK_FALSE || PDF.sampleRateShading) && "");
	VERIFY((PMSCI.minSampleShading >= 0.0f && PMSCI.minSampleShading <= 1.0f));
	//assert((PMSCI.alphaToOneEnable == VK_FALSE || PDF.alphaToOne) && "");

	//!< カラーブレンド (ColorBlend)
	//!< VK_BLEND_FACTOR_SRC1 系をを使用するには、デバイスフィーチャー dualSrcBlend が有効であること
	///!< SRCコンポーネント * SRCファクタ OP DSTコンポーネント * DSTファクタ
	//!< デバイスフィーチャー independentBlend が有効で無い場合は、配列の各要素は「完全に同じ値」であること (If device feature independentBlend is not enabled, each array element must be exactly same)
	//if (!PDF.independentBlend) {
	//	for (auto i : PCBASs) {
	//		assert(memcmp(&i, &PCBASs[0], sizeof(PCBASs[0])) == 0 && ""); //!< 最初の要素は比べる必要無いがまあいいや
	//	}
	//}
	const VkPipelineColorBlendStateCreateInfo PCBSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.logicOpEnable = VK_FALSE, .logicOp = VK_LOGIC_OP_COPY, //!< ブレンド時に論理オペレーションを行う (ブレンドは無効になる) (整数型アタッチメントに対してのみ)
		.attachmentCount = static_cast<uint32_t>(std::size(PCBASs)), .pAttachments = std::data(PCBASs),
		.blendConstants = { 1.0f, 1.0f, 1.0f, 1.0f }
	};

	//!< ダイナミックステート (DynamicState)
	constexpr std::array DSs = {
		VK_DYNAMIC_STATE_VIEWPORT,
		VK_DYNAMIC_STATE_SCISSOR,
		//VK_DYNAMIC_STATE_DEPTH_BIAS,
	};
	const VkPipelineDynamicStateCreateInfo PDSCI = {
		.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
		.pNext = nullptr,
		.flags = 0,
		.dynamicStateCount = static_cast<uint32_t>(std::size(DSs)), .pDynamicStates = std::data(DSs)
	};

	/**
	@brief 継承 ... 共通部分が多い場合、親パイプラインを指定して作成するとより高速に作成できる、親子間でのスイッチやバインドが有利
	(DX の D3D12_CACHED_PIPELINE_STATE 相当?)
	basePipelineHandle, basePipelineIndex は同時に使用できない、使用しない場合はそれぞれ VK_NULL_HANDLE, -1 を指定すること
	親には VK_PIPELINE_CREATE_ALLOW_DERIVATIVES_BIT フラグが必要、子には VK_PIPELINE_CREATE_DERIVATIVE_BIT フラグが必要
	・basePipelineHandle ... 既に親とするパイプライン(ハンドル)が存在する場合に指定
	・basePipelineIndex ... 同配列内で親パイプラインも同時に作成する場合、配列内での親パイプラインの添字(親の添字の方が若い値であること)
	*/
	const std::array GPCIs = {
		VkGraphicsPipelineCreateInfo({
			.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
			.pNext = nullptr,
#ifdef _DEBUG
			.flags = VK_PIPELINE_CREATE_DISABLE_OPTIMIZATION_BIT,
#else
			.flags = 0,
#endif
			.stageCount = static_cast<uint32_t>(std::size(PSSCIs)), .pStages = std::data(PSSCIs),
			.pVertexInputState = &PVISCI,
			.pInputAssemblyState = &PIASCI,
			.pTessellationState = &PTSCI,
			.pViewportState = &PVSCI,
			.pRasterizationState = &PRSCI,
			.pMultisampleState = &PMSCI,
			.pDepthStencilState = &PDSSCI,
			.pColorBlendState = &PCBSCI,
			.pDynamicState = &PDSCI,
			.layout = PLL,
			.renderPass = RP, .subpass = 0, //!< ここで指定するレンダーパスは「互換性のあるもの」なら可
			.basePipelineHandle = VK_NULL_HANDLE, .basePipelineIndex = -1
		})
	};
	//!< VKでは1コールで複数のパイプラインを作成することもできるが、DXに合わせて1つしか作らないことにしておく
	VERIFY_SUCCEEDED(vkCreateGraphicsPipelines(Dev, VK_NULL_HANDLE, static_cast<uint32_t>(std::size(GPCIs)), std::data(GPCIs), GetAllocationCallbacks(), &PL));
}

void VK::CreateViewport(const FLOAT Width, const FLOAT Height, const FLOAT MinDepth, const FLOAT MaxDepth)
{
	Viewports = {
			VkViewport({
			//!< VKではデフォルトで「Yが下」を向くが、高さに負の値を指定すると「Yが上」を向き、DXと同様になる (In VK, by specifying negative height, Y become up. same as DX)
			//!< 通常基点は「左上」を指定するが、高さに負の値を指定する場合は「左下」を指定すること (When negative height, specify left bottom as base, otherwise left up)
			.x = 0.0f, .y = Height,
			.width = Width, .height = -Height,
			.minDepth = MinDepth, .maxDepth = MaxDepth
		})
	};
	//!< offset, extent で指定 (left, top, right, bottomで指定のDXとは異なるので注意)
	ScissorRects = {
		VkRect2D({.offset = VkOffset2D({.x = 0, .y = 0 }), .extent = VkExtent2D({.width = static_cast<uint32_t>(Width), .height = static_cast<uint32_t>(Height) }) }),
	};
}

void VK::WaitForFence(VkDevice Device, VkFence Fence) 
{
	const std::array Fences = { Fence };
	VERIFY_SUCCEEDED(vkWaitForFences(Device, static_cast<uint32_t>(std::size(Fences)), std::data(Fences), VK_TRUE, (std::numeric_limits<uint64_t>::max)()));
	vkResetFences(Device, static_cast<uint32_t>(std::size(Fences)), std::data(Fences));
}
void VK::SubmitGraphics(const uint32_t i)
{
	const std::array WaitSems = { NextImageAcquiredSemaphore };
	const std::array WaitStages = { VkPipelineStageFlags(VK_PIPELINE_STAGE_TRANSFER_BIT) };
	//!< 実行するコマンドバッファ
	const std::array CBs = { CommandBuffers[i], };
	//!< 完了時にシグナルされるセマフォ (RenderFinishedSemaphore) -> これを待ってからプレゼントが行われる
	const std::array SigSems = { RenderFinishedSemaphore };
	const std::array SIs = {
		VkSubmitInfo({
			.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
			.pNext = nullptr,
			.waitSemaphoreCount = static_cast<uint32_t>(std::size(WaitSems)), .pWaitSemaphores = std::data(WaitSems), .pWaitDstStageMask = std::data(WaitStages), //!< 次イメージが取得できる(プレゼント完了)までウエイト
			.commandBufferCount = static_cast<uint32_t>(std::size(CBs)), .pCommandBuffers = std::data(CBs),
			.signalSemaphoreCount = static_cast<uint32_t>(std::size(SigSems)), .pSignalSemaphores = std::data(SigSems) //!< 描画完了を通知する
		}),
	};
	VERIFY_SUCCEEDED(vkQueueSubmit(GraphicsQueue, static_cast<uint32_t>(std::size(SIs)), std::data(SIs), GraphicsFence));
}
void VK::Present() 
{
	const std::array WaitSems = { RenderFinishedSemaphore };
	//!< 同時に複数のプレゼントが可能だが、1つのスワップチェインからは1つのみ
	const std::array Swapchains = { Swapchain };
	const std::array ImageIndices = { SwapchainImageIndex };
	VERIFY(size(Swapchains) == size(ImageIndices));

	//!< サブミット時に指定したセマフォ(RenderFinishedSemaphore)を待ってからプレゼントが行なわれる
	const VkPresentInfoKHR PresentInfo = {
		.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
		.pNext = nullptr,
		.waitSemaphoreCount = static_cast<uint32_t>(std::size(WaitSems)), .pWaitSemaphores = std::data(WaitSems),
		.swapchainCount = static_cast<uint32_t>(std::size(Swapchains)), .pSwapchains = std::data(Swapchains), .pImageIndices = std::data(ImageIndices),
		.pResults = nullptr
	};
	VERIFY_SUCCEEDED(vkQueuePresentKHR(PresentQueue, &PresentInfo));
}
void VK::Draw() 
{
	WaitForFence(Device, GraphicsFence);

	VERIFY_SUCCEEDED(vkAcquireNextImageKHR(Device, Swapchain, UINT64_MAX, NextImageAcquiredSemaphore, VK_NULL_HANDLE, &SwapchainImageIndex));
	DrawFrame(SwapchainImageIndex);
	SubmitGraphics(SwapchainImageIndex);

	Present();
}