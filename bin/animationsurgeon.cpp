/*
 * Copyright (c) 2017 Softbank Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#include <boost/program_options.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <almath/scenegraph/qianim/surgeon.h>
#include <iostream>

using namespace AL::qianim;
namespace po = boost::program_options;
namespace xml_parser = boost::property_tree::xml_parser;
namespace bfs = boost::filesystem;

void apply_commands(const po::variables_map &vm, ptree &docroot,
                    boost::optional<bfs::path> path=boost::optional<bfs::path>()) {
  ptree &animation = AL::qianim::V2::get_animation(docroot);
  if (vm.count("fix-unit")) {
    for(ptree &curve: V2::Animation::get_actuatorcurves(animation))
      ActuatorCurve::fix_missing_unit(curve, true);
  }
  if (vm.count("to-degree")) {
      for(ptree &curve: V2::Animation::get_actuatorcurves(animation))
        ActuatorCurve::convert_unit(curve, Unit::degree, Unit::radian);
  }
  if (vm.count("to-radian")) {
    for(ptree &curve: V2::Animation::get_actuatorcurves(animation))
      ActuatorCurve::convert_unit(curve, Unit::radian, Unit::degree);
  }
  if (vm.count("offset-frame")) {
    const auto offset = vm["offset-frame"].as<int>();
    auto tr = [offset] (ptree &curve) {
        boost::for_each(V2::ActuatorCurve::get_keys(curve),
                        [offset] (ptree &key) {Key::offset_frame(key, offset);});};
    boost::for_each(V2::Animation::get_actuatorcurves(animation), tr);
  }
  if (vm.count("check"))
    V2::Animation::check_all(animation);

  xml_parser::xml_writer_settings<std::string> xml_settings(' ', 2);
  if (vm.count("print")) {
    xml_parser::write_xml(std::cout, docroot, xml_settings);
  }

  if (vm.count("inplace") && path) {
    // write back result in the same file
    assert(!vm.count("migrate-xar"));
    bfs::ofstream out(*path);
    xml_parser::write_xml(out, docroot, xml_settings);
  }
}

int main(int argc, char *argv[])
{
  std::vector<std::string> files;

  po::options_description generic("");
  generic.add_options()
  ("file", po::value< std::vector<std::string> >(&files),
   "path to input file");

  po::options_description command("Commands");
  command.add_options()
  ("help",
   "display help message")
  ("migrate-xar",
   "migrate all timeline animations from a xar to v2.0 format")
  ("migrate-v1",
   "migrate legacy v1.0 animation to v2.0 format")
  ("fix-unit",
   "fix missing unit attributes,"
    " using \"dimensionless\" if the curve is named"
    " \"LHand\" or \"RHand\", \"degree\" otherwise.")
  ("to-degree",
   "convert curves in radian to degree")
  ("to-radian",
   "convert curves in degree to radian")
  ("offset-frame",
   po::value<int>(),
   "add the given offset to each key frame")
  ("check",
   "check the file is a valid v2.0 animation")
  ("print",
   "print the animation to cout")
  ("inplace",
   "write back the animation inplace (incompatible with --migrate-xar)");

  po::positional_options_description positional;
  positional.add("file", -1);

  po::options_description visible(
      "Modify animation xml files.\n\n"
      "Usage:\n animationsurgeon --migrate-xar --check --print file0 file1\n\n"
      "Options");
  visible.add(generic);
  visible.add(command);

  int skippedFiles = 0;
  try
  {
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(visible).positional(positional).run(), vm);
    if (vm.count("help"))
    {
      std::cout << visible << "\n";
      return 0;
    }
    po::notify(vm);

    for (auto &&file : files) {
      bfs::path path(file);
      try {
        // open the file
        ptree docroot;
        bfs::ifstream in(path);
        xml_parser::read_xml(in, docroot, xml_parser::trim_whitespace);

        // first deal with conversions to v2
        if (vm.count("migrate-xar")) {
          for (auto && root : v2_roots_from_xar(docroot)) {
            apply_commands(vm, root);
          }
        } else if (vm.count("migrate-v1")) {
          docroot = v2_root_from_v1_root(docroot);
          apply_commands(vm, docroot, path);
        } else {
          apply_commands(vm, docroot, path);
        }
      } catch (std::exception &e) {
        std::cerr << "Error on file " << path << ": " << e.what() << "\n";
      }
    }
  } catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
  return skippedFiles;
}

