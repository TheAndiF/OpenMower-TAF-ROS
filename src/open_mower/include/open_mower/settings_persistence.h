#pragma once

#include <cerrno>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

namespace open_mower_settings {
using json = nlohmann::ordered_json;

class FileLock {
 public:
  explicit FileLock(const std::string& lock_path) : fd_(-1) {
    fd_ = ::open(lock_path.c_str(), O_CREAT | O_RDWR, 0644);
    if (fd_ >= 0) {
      if (::flock(fd_, LOCK_EX) != 0) {
        ::close(fd_);
        fd_ = -1;
      }
    }
  }

  ~FileLock() {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  bool locked() const { return fd_ >= 0; }

 private:
  int fd_;
};

inline void ensureDataRosDirectory() {
  ::mkdir("/data", 0755);
  ::mkdir("/data/ros", 0755);
}

inline json emptySettingsRoot() {
  json root = json::object();
  root["schema"] = "settings_persistent_v1";
  root["settings"] = json::object();
  return root;
}

inline json normalizeRoot(json root) {
  if (!root.is_object()) {
    root = emptySettingsRoot();
  }
  if (!root.contains("schema") || !root["schema"].is_string()) {
    root["schema"] = "settings_persistent_v1";
  }
  if (!root.contains("settings") || !root["settings"].is_object()) {
    root["settings"] = json::object();
  }
  return root;
}

inline json readRootUnlocked(const std::string& path) {
  std::ifstream in(path);
  if (!in.good()) {
    return emptySettingsRoot();
  }
  try {
    json parsed;
    in >> parsed;
    return normalizeRoot(parsed);
  } catch (...) {
    return emptySettingsRoot();
  }
}

inline bool writeRootUnlocked(const std::string& path, const json& root) {
  const std::string temp_path = path + ".tmp";
  {
    std::ofstream out(temp_path, std::ios::trunc);
    if (!out.good()) {
      return false;
    }
    out << root.dump(2) << std::endl;
    if (!out.good()) {
      return false;
    }
  }
  return std::rename(temp_path.c_str(), path.c_str()) == 0;
}

inline json mergeNamespaceWithSeed(const std::string& path, const std::string& namespace_name,
                                   const json& seed_entries, bool* wrote_file = nullptr) {
  ensureDataRosDirectory();
  FileLock lock(path + ".lock");
  json root = readRootUnlocked(path);
  json before = root;

  if (!root["settings"].contains(namespace_name) || !root["settings"][namespace_name].is_object()) {
    root["settings"][namespace_name] = json::object();
  }
  json& namespace_entries = root["settings"][namespace_name];

  for (auto it = seed_entries.begin(); it != seed_entries.end(); ++it) {
    const std::string key = it.key();
    const json& seed_entry = it.value();
    if (!namespace_entries.contains(key) || !namespace_entries[key].is_object()) {
      namespace_entries[key] = seed_entry;
      continue;
    }
    json& existing_entry = namespace_entries[key];
    for (auto seed_field = seed_entry.begin(); seed_field != seed_entry.end(); ++seed_field) {
      if (!existing_entry.contains(seed_field.key())) {
        existing_entry[seed_field.key()] = seed_field.value();
      }
    }
  }

  const bool changed = (root != before);
  if (changed) {
    writeRootUnlocked(path, root);
  }
  if (wrote_file) {
    *wrote_file = changed;
  }
  return namespace_entries;
}

inline json readNamespace(const std::string& path, const std::string& namespace_name) {
  ensureDataRosDirectory();
  FileLock lock(path + ".lock");
  const json root = readRootUnlocked(path);
  if (!root.contains("settings") || !root["settings"].is_object()) {
    return json::object();
  }
  if (!root["settings"].contains(namespace_name) || !root["settings"][namespace_name].is_object()) {
    return json::object();
  }
  return root["settings"][namespace_name];
}

inline bool updateEntryField(const std::string& path, const std::string& namespace_name,
                             const std::string& key, const std::string& field, const json& value) {
  ensureDataRosDirectory();
  FileLock lock(path + ".lock");
  json root = readRootUnlocked(path);
  if (!root["settings"].contains(namespace_name) || !root["settings"][namespace_name].is_object()) {
    root["settings"][namespace_name] = json::object();
  }
  json& namespace_entries = root["settings"][namespace_name];
  if (!namespace_entries.contains(key) || !namespace_entries[key].is_object()) {
    namespace_entries[key] = json::object();
  }
  namespace_entries[key][field] = value;
  return writeRootUnlocked(path, root);
}

inline bool isNumber(const json& object, const std::string& field) {
  return object.is_object() && object.contains(field) && object[field].is_number();
}

inline bool isBoolean(const json& object, const std::string& field) {
  return object.is_object() && object.contains(field) && object[field].is_boolean();
}

inline double numberOr(const json& object, const std::string& field, double fallback) {
  return isNumber(object, field) ? object[field].get<double>() : fallback;
}

inline bool boolOr(const json& object, const std::string& field, bool fallback) {
  return isBoolean(object, field) ? object[field].get<bool>() : fallback;
}

inline std::string stringOr(const json& object, const std::string& field, const std::string& fallback) {
  return object.is_object() && object.contains(field) && object[field].is_string()
             ? object[field].get<std::string>()
             : fallback;
}

inline int intOr(const json& object, const std::string& field, int fallback) {
  return object.is_object() && object.contains(field) && object[field].is_number_integer()
             ? object[field].get<int>()
             : fallback;
}

}  // namespace open_mower_settings
