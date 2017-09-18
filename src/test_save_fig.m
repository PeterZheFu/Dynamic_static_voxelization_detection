h = figure;
st.dr.save = '~/continental/kitti/2011_09_26/four_image_results_all_seq';
fpath = st.dr.save
fname = 'test';
saveas(h, fullfile(fpath, fname));