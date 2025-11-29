import React from 'react';
import Layout from '@theme/Layout';
import '../css/custom.css';

export default function BonusUrdu() {
  return (
    <Layout title="اردو گائیڈ" description="اردو زبان میں رہنمائی">
      <div className="bonus-page">
        <div className="bonus-content">
          <h1 className="bonus-title urdu-title">اردو گائیڈ</h1>
          
          <div className="bonus-section urdu-text">
            <h2>تعارف</h2>
            <p>
              یہ اردو زبان میں لکھا گیا ایک خصوصی گائیڈ ہے۔ یہاں آپ کو اردو میں مختلف موضوعات 
              پر معلومات ملے گی۔
            </p>
          </div>
          
          <div className="bonus-section urdu-text">
            <h2>اہم موضوعات</h2>
            <ul>
              <li>آڈینٹیکیشن کے طریقے</li>
              <li>ذاتی نوعیت کی ترتیبات</li>
              <li>صارف کا تجربہ</li>
              <li>سیکیورٹی کے اصول</li>
              <li>جدید ٹیکنالوجیز</li>
            </ul>
          </div>
          
          <div className="bonus-section urdu-text">
            <h2>فوائد</h2>
            <p>
              اردو زبان میں مواد پڑھنے سے صارفین کو بہتر سمجھ آتی ہے اور وہ زیادہ آرام دہ محسوس کرتے ہیں۔ 
              یہ مقامی زبان میں تعلیم کو فروغ دیتا ہے۔
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}